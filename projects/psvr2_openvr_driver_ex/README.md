# DriverEx
PlayStation VR2 Toolkit's "driver extension" for the official PS VR2 driver from Sony.

# Project Structure
- `/*_proxy(.h/.cpp)`: Proxy classes for the OpenVR interfaces, allowing DriverEx to intercept things like pose data before its actually sent to SteamVR, as well as initialize our hooks and extensions before the official PS VR2 driver is loaded.
- `/driver_hooks/`: Hooks for the original PS VR2 driver, adding extra functionality on top of the original PS VR2 driver.
- `/driver_interface/`: Interfacing headers for the original PS VR2 driver, such as structures, classes, functions, etc.
- `/utils/`: General-purpose internal utilities, for usage within DriverEx.
