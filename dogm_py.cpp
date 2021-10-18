
#include "./pybind11/include/pybind11/pybind11.h"
#include "./pybind11/include/pybind11/stl_bind.h"
#include "./pybind11/include/pybind11/numpy.h"

#include <dogm/dogm.h>
#include <dogm/dogm_types.h>
#include <dogm/mapping/laser_to_meas_grid.h>
#include <iostream>

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

  py::array_t<float> render_occupancy_grid( const DOGM& grid ) {
    auto width = grid.getGridSize();
    auto height = grid.getGridSize();
    float *grid_ptr = new float[ width * height ];

    const auto grid_cells = grid.getGridCells();
    #pragma omp parallel for
    for( int y = 0; y < height; y++ ) {
      for( int x = 0; x < width; x++ ) {
        auto cell = grid_cells[y*width + x];
        float free_mass = cell.free_mass;
        float occ_mass = cell.occ_mass;

        float prob = occ_mass + 0.5f * (1.0f - occ_mass - free_mass);
        grid_ptr[ y*width + x ] = prob;
      }
    }

    // Create a Python object that will free the allocated
    // memory when destroyed:
    py::capsule free_when_done(grid_ptr, [](void *p) {
      double *ptr = reinterpret_cast<double *>(p);
      std::cerr << "Element [0] = " << ptr[0] << "\n";
      std::cerr << "freeing memory @ " << p << "\n";
      delete[] ptr;
    });

    return py::array_t<float>(
            {width, height}, // shape
            {width*sizeof(float), sizeof(float)}, // C-style contiguous strides
            grid_ptr, // the data pointer
            free_when_done); // numpy array references this parent
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
            .def(py::init<float, float, float>(),
                 R"lmgPARAM(
    struct Params {
      float max_range;
      float resolution;
      float fov;
    };
                        )lmgPARAM", py::arg("max_range"), py::arg("resolution"),
                        py::arg("fov"))
            .def_readwrite("max_range", &LaserMeasurementGrid::Params::max_range)
            .def_readwrite("resolution", &LaserMeasurementGrid::Params::resolution)
            .def_readwrite("fov", &LaserMeasurementGrid::Params::fov)
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

  }


}


