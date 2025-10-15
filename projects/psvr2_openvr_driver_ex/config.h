#pragma once

#define DRIVER_IS_STABLE false
#define DRIVER_IS_PRERELEASE true
#define DRIVER_IS_EXPERIMENTAL false

#define DRIVER_VERSION_MAJOR 0
#define DRIVER_VERSION_MINOR 2
#define DRIVER_VERSION_PATCH 0

#if DRIVER_IS_STABLE
#define DRIVER_VERSION_BRANCH "stable"
#elif DRIVER_IS_PRERELEASE
#define DRIVER_VERSION_BRANCH "prerelease"
#elif DRIVER_IS_EXPERIMENTAL
#define DRIVER_VERSION_BRANCH "experimental"
#else
#define DRIVER_VERSION_BRANCH "development"
#endif

// Whether or not we should mock/fake running the driver on Wine.
#define MOCK_IS_RUNNING_ON_WINE false
