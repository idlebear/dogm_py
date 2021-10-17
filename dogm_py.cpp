
#include "./pybind11/include/pybind11/pybind11.h"

#include <dogm/dogm.h>
#include <dogm/dogm_types.h>
#include <dogm/mapping/laser_to_meas_grid.h>

namespace py = pybind11;

namespace dogm {

  PYBIND11_MODULE(dogm_py, m) {
    m.doc() = "Python bindings for the Dynamic Occupancy Grid Map Library";

    py::class_<LaserMeasurementGrid::Params>(m, "LaserMeasurementGridParams")
            .def(py::init<float, float, float>(),
                 R"lmgPARAM(
    struct Params {
      float max_range;
      float resolution;
      float fov;
    };
                        )lmgPARAM", py::arg("max_range"), py::arg("resolution"),
                        py::arg("fov"));

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
  }

}


