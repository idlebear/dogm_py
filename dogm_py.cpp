
#include "./pybind11/include/pybind11/pybind11.h"
#include "./pybind11/include/pybind11/stl_bind.h"
#include "./pybind11/include/pybind11/numpy.h"

#include <algorithm>
#include <dogm/dogm.h>
#include <dogm/dogm_types.h>
#include <dogm/mapping/laser_to_meas_grid.h>
#include <iostream>

#include <Eigen/Dense>

namespace py = pybind11;

PYBIND11_MAKE_OPAQUE(std::vector<float>);
PYBIND11_MAKE_OPAQUE(std::vector<dogm::GridCell>);

namespace dogm {

  // https://stackoverflow.com/questions/48982143/returning-and-passing-around-raw-pod-pointers-arrays-with-python-c-and-pyb
  template <class T> class ptr_wrapper
  {
  public:
    ptr_wrapper() : ptr(nullptr) {}
    ptr_wrapper(T* ptr) : ptr(ptr) {}
    ptr_wrapper(const ptr_wrapper& other) : ptr(other.ptr) {}
    T& operator* () const { return *ptr; }
    T* operator->() const { return  ptr; }
    T* get() const { return ptr; }
    void destroy() { delete ptr; }
    T& operator[](std::size_t idx) const { return ptr[idx]; }
  private:
    T* ptr;
  };

  ptr_wrapper<MeasurementCell> wrap_laser_grid( LaserMeasurementGrid& lmg, const std::vector<float>& measurements ) {
    ptr_wrapper<MeasurementCell> ptr(lmg.generateGrid( measurements ));
    return ptr;
  }

  void wrap_dogm_grid( DOGM& grid, ptr_wrapper<MeasurementCell>& ptr, float x, float y, float yaw, float dt, bool device){
    grid.updateGrid( ptr.get(), x, y, yaw, dt, device );
  }

  float pignisticTransformation(float free_mass, float occ_mass) {
    return occ_mass + 0.5f * (1.0f - occ_mass - free_mass);
  }

  void hsvToRGB(float hue, float saturation, float value, float &r, float &g, float &b) {

    if (saturation == 0.0f) {
      r = g = b = value;
    } else {
      int i = static_cast<int>(hue * 6.0f);
      float f = (hue * 6.0f) - i;
      float p = value * (1.0f - saturation);
      float q = value * (1.0f - saturation * f);
      float t = value * (1.0f - saturation * (1.0f - f));
      int res = i % 6;

      switch (res) {
        case 0:
          r = value;
          g = t;
          b = p;
          break;
        case 1:
          r = q;
          g = value;
          b = p;
          break;
        case 2:
          r = p;
          g = value;
          b = t;
          break;
        case 3:
          r = p;
          g = q;
          b = value;
          break;
        case 4:
          r = t;
          g = p;
          b = value;
          break;
        case 5:
          r = value;
          g = p;
          b = q;
          break;
        default:
          r = g = b = value;
      }
    }
  }

  py::array_t<float> render_dynamic_occupancy_grid( const DOGM& grid, double occupancy_threshold,
                                                            double dynamic_threshold, double max_velocity ) {
    auto width = grid.getGridSize();
    auto height = grid.getGridSize();
    auto depth = 3;
    float *grid_ptr = new float[ depth * width * height ];

    const auto grid_cells = grid.getGridCells();
    #pragma omp parallel for
    for( int y = 0; y < height; y++ ) {
      for( int x = 0; x < width; x++ ) {
        auto cell = grid_cells[y*width + x];
        float occ = pignisticTransformation(cell.free_mass, cell.occ_mass);

        Eigen::Vector2f vel;
        vel << cell.mean_x_vel, cell.mean_y_vel;

        Eigen::Matrix2f covar;
        covar << cell.var_x_vel, cell.covar_xy_vel, cell.covar_xy_vel, cell.var_y_vel;

        auto mdist = vel.transpose() * covar.inverse() * vel;

        float r, g, b;
        if (occ >= occupancy_threshold && mdist >= dynamic_threshold ) {
          float angle = atan2(cell.mean_y_vel, cell.mean_x_vel) + M_PI;

          auto value = std::min(1.0, sqrt(cell.mean_x_vel * cell.mean_x_vel +
                                          cell.mean_y_vel * cell.mean_y_vel) / max_velocity );

          hsvToRGB(angle / M_PI, 1.0f, value, r, g, b);
        } else {
          r = g = b = 1.0f - occ;
        }
        grid_ptr[ depth * ( y*width + x ) ] = r;
        grid_ptr[ depth * ( y*width + x ) + 1 ] = g;
        grid_ptr[ depth * ( y*width + x ) + 2 ] = b;
      }
    }

    // Create a Python object that will free the allocated
    // memory when destroyed:
    py::capsule delete_fn(grid_ptr, [](void *p) {
      float *ptr = reinterpret_cast<float *>(p);
      delete[] ptr;
    });

    return py::array_t<float>(
            {height, width, depth}, // shape
            {depth*width*sizeof(float), depth*sizeof(float), sizeof(float)}, // C-style contiguous strides
            grid_ptr, // the data pointer
            delete_fn); // numpy array references this parent
  }


  py::array_t<float> render_occupancy_grid( const DOGM& grid ) {
    auto width = grid.getGridSize();
    auto height = grid.getGridSize();
    float *grid_ptr = new float[ width * height ];

    const auto grid_cells = grid.getGridCells();
  #pragma omp parallel for
    for( int y = 0; y < height; y++ ) {
      for( int x = 0; x < width; x++ ) {
        auto cell = grid_cells[y*width + x];
        float prob = pignisticTransformation( cell.free_mass, cell.occ_mass );
        grid_ptr[ y*width + x ] = 1.0f - prob;
      }
    }

    // Create a Python object that will free the allocated
    // memory when destroyed:
    py::capsule delete_fn(grid_ptr, [](void *p) {
      float *ptr = reinterpret_cast<float *>(p);
      delete[] ptr;
    });

    return py::array_t<float>(
            {height, width}, // shape
            {width*sizeof(float), sizeof(float)}, // C-style contiguous strides
            grid_ptr, // the data pointer
            delete_fn); // numpy array references this parent
  }


  PYBIND11_MODULE(dogm_py, m) {
    // bind a vector of floats to pass into the laser measurement grid
    py::bind_vector<std::vector<float>>(m, "VectorFloat", py::buffer_protocol());

    m.doc() = "Python bindings for the Dynamic Occupancy Grid Map Library";

    py::class_<ptr_wrapper<MeasurementCell>>(m, "MeasurementCellPtr")
      .def(py::init<>(), "Wrapper for CUDA pointer returned by LaserMeasurementGrid" );

    m.def( "generateMeasurements", &wrap_laser_grid, py::arg( "laser_measurement_grid"),
           py::arg("measurements") );

    m.def( "updateGrid", &wrap_dogm_grid, "Wrapper function for grid update",
           py::arg("grid"), py::arg("measurement_ptr"), py::arg("x"),
           py::arg("y"), py::arg("yaw"),
           py::arg("dt"), py::arg("device"));

    py::class_<LaserMeasurementGrid::Params>(m, "LaserMeasurementGridParams")
            .def(py::init<float, float, float, float>(),
                 R"lmgPARAM(
    struct Params {
      float max_range;
      float resolution;
      float fov;
      float angle_increment;
    };
                        )lmgPARAM", py::arg("max_range"), py::arg("resolution"),
                        py::arg("fov"), py::arg("angle_increment"))
            .def_readwrite("max_range", &LaserMeasurementGrid::Params::max_range)
            .def_readwrite("resolution", &LaserMeasurementGrid::Params::resolution)
            .def_readwrite("fov", &LaserMeasurementGrid::Params::fov)
            .def_readwrite("angle_increment", &LaserMeasurementGrid::Params::angle_increment)
            ;

    py::class_<LaserMeasurementGrid>(m, "LaserMeasurementGrid")
            .def(py::init<const LaserMeasurementGrid::Params&, float, float>(),
                 py::arg("params"), py::arg("size"), py::arg("resolution"))
            .def("generateGrid", &LaserMeasurementGrid::generateGrid, "Create a grid representation of supplied laser measurements",
                 py::arg("measurements"));

    py::class_<DOGM::Params>(m, "DOGMParams")
            .def(py::init<float, float, int, int, float, float, float, float, float, float>(),
                    R"DogmPARAM(
    struct Params {
      float size;                          // Grid size [m]
      float resolution;                    // Grid cell size [m/cell]
      int particle_count;                  // Number of persistent particles
      int new_born_particle_count;         // Number of birth particles
      float persistence_prob;              // Probability of persistence
      float stddev_process_noise_position; // Process noise position
      float stddev_process_noise_velocity; // Process noise velocity
      float birth_prob;                    // Probability of birth
      float stddev_velocity;               // Velocity to sample birth particles from (normal distribution) [m/s]
      float init_max_velocity;             // Velocity to sample the initial particles from (uniform distribution) [m/s]
    }
                        )DogmPARAM", py::arg("size"),py::arg("resolution"),py::arg("particle_count"),
                   py::arg("new_born_particle_count"),py::arg("persistance_prob"),py::arg("stddev_process_noise_position"),
                   py::arg("stddev_process_noise_velocity"), py::arg("birth_prob"), py::arg("stddev_velocity"),
                   py::arg("init_max_velocity"));

    // define a vector of GridCells for return types
    py::class_<GridCell>(m, "GridCell")
            .def(py::init<>())
            .def_readwrite("occ_mass", &GridCell::occ_mass)
            .def_readwrite("free_mass", &GridCell::free_mass);

    py::bind_vector<std::vector<GridCell>>(m, "VectorGridCell");

    py::class_<DOGM>(m, "DOGM")
            .def(py::init<const DOGM::Params&>(), py::arg("params") )
            .def("updateGrid", &DOGM::updateGrid, "Update the current state with constructed measurement grid",
                 py::arg("cellData"), py::arg("x"), py::arg("y"), py::arg("yaw"), py::arg("dt"), py::arg("device"))
            .def("getGridCells", &DOGM::getGridCells)
            .def("getMeasurementCells", &DOGM::getMeasurementCells)
            .def("getParticles", &DOGM::getParticles)
            .def("getGridSize", &DOGM::getGridSize)
            .def("getResolution", &DOGM::getResolution)
            .def("getPositionX", &DOGM::getPositionX)
            .def("getPositionY", &DOGM::getPositionY)
            .def("getYaw", &DOGM::getYaw)
            .def("getIteration", &DOGM::getIteration);

    m.def( "renderOccupancyGrid", &render_occupancy_grid, "Convert the occupancy grid into a numpy array",
           py::arg("grid"));

    m.def( "renderDynamicOccupancyGrid", &render_dynamic_occupancy_grid, "Convert the occupancy grid into a RGB numpy array encoding of dynamic elements",
           py::arg("grid"), py::arg("occupancy_threshold"), py::arg("dynamic_threshold"), py::arg("max_velocity"));

  }


}


