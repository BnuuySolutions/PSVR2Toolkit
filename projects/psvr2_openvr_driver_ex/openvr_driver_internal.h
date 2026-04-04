#pragma once

#include <openvr_driver.h>

namespace vr {

  enum EBlockQueueError {
    BlockQueueError_None = 0,
    BlockQueueError_QueueAlreadyExists = 1,
    BlockQueueError_QueueNotFound = 2,
    BlockQueueError_BlockNotAvailable = 3,
    BlockQueueError_InvalidHandle = 4,
    BlockQueueError_InvalidParam = 5,
    BlockQueueError_ParamMismatch = 6,
    BlockQueueError_InternalError = 7,
    BlockQueueError_AlreadyInitialized = 8,
    BlockQueueError_OperationIsServerOnly = 9,
    BlockQueueError_TooManyConnections = 10,
  };

  enum EBlockQueueReadType {
    BlockQueueRead_Latest = 0,
    BlockQueueRead_New = 1,
    BlockQueueRead_Next = 2,
  };
  
  enum EBlockQueueCreationFlag {
    BlockQueueFlag_OwnerIsReader = 1,
  };

  class IVRBlockQueue {
  public:
    virtual EBlockQueueError Create(PropertyContainerHandle_t *pulQueueHandle, const char *pchPath, uint32_t unBlockDataSize, uint32_t unBlockHeaderSize, uint32_t unBlockCount, uint32_t unFlags = 0) = 0;
    virtual EBlockQueueError Connect(PropertyContainerHandle_t *pulQueueHandle, const char *pchPath) = 0;
    virtual EBlockQueueError Destroy(PropertyContainerHandle_t ulQueueHandle) = 0;
    virtual EBlockQueueError AcquireWriteOnlyBlock(PropertyContainerHandle_t ulQueueHandle, PropertyContainerHandle_t *pulBlockHandle, void **ppvBuffer) = 0;
    virtual EBlockQueueError ReleaseWriteOnlyBlock(PropertyContainerHandle_t ulQueueHandle, PropertyContainerHandle_t ulBlockHandle) = 0;
    virtual EBlockQueueError WaitAndAcquireReadOnlyBlock(PropertyContainerHandle_t ulQueueHandle, PropertyContainerHandle_t *pulBlockHandle, void **ppvBuffer, EBlockQueueReadType eReadType, uint32_t unTimeoutMs) = 0;
    virtual EBlockQueueError AcquireReadOnlyBlock(PropertyContainerHandle_t ulQueueHandle, PropertyContainerHandle_t *pulBlockHandle, void **ppvBuffer, EBlockQueueReadType eReadType) = 0;
    virtual EBlockQueueError ReleaseReadOnlyBlock(PropertyContainerHandle_t ulQueueHandle, PropertyContainerHandle_t ulBlockHandle) = 0;
    virtual EBlockQueueError QueueHasReader(PropertyContainerHandle_t ulQueueHandle, bool *pbHasReaders) = 0;
  };

  static const char *IVRBlockQueue_Version = "IVRBlockQueue_005";

  typedef uint64_t PathHandle_t;
  
  struct PathWrite_t {
    PathHandle_t ulPath;
    enum EPropertyWriteType writeType;
    enum ETrackedPropertyError eSetError;
    void *pvBuffer; // void *
    uint32_t unBufferSize;
    PropertyTypeTag_t unTag;
    enum ETrackedPropertyError eError;
    char *pszPath; // const char *
  };

  struct PathRead_t {
    PathHandle_t ulPath;
    void *pvBuffer; // void *
    uint32_t unBufferSize;
    PropertyTypeTag_t unTag;
    uint32_t unRequiredBufferSize;
    enum ETrackedPropertyError eError;
    char *pszPath; // const char *
  };

  class IVRPaths {
  public:
    virtual ETrackedPropertyError ReadPathBatch(PropertyContainerHandle_t ulRootHandle, struct PathRead_t *pBatch, uint32_t unBatchEntryCount) = 0;
    virtual ETrackedPropertyError WritePathBatch(PropertyContainerHandle_t ulRootHandle, struct PathWrite_t *pBatch, uint32_t unBatchEntryCount) = 0;
    virtual ETrackedPropertyError StringToHandle(PathHandle_t *pHandle, const char *pchPath) = 0;
    virtual ETrackedPropertyError HandleToString(PathHandle_t pHandle, char *pchBuffer, uint32_t unBufferSize, uint32_t *punBufferSizeUsed) = 0;
  };

  static const char *IVRPaths_Version = "IVRPaths_001";

  template <typename T>
  struct VrPathTagTraits;

  template <> struct VrPathTagTraits<uint64_t> { static constexpr vr::PropertyTypeTag_t tag = vr::k_unUint64PropertyTag; };
  template <> struct VrPathTagTraits<int32_t> { static constexpr vr::PropertyTypeTag_t tag = vr::k_unInt32PropertyTag; };
  template <> struct VrPathTagTraits<double> { static constexpr vr::PropertyTypeTag_t tag = vr::k_unDoublePropertyTag; };
  template <> struct VrPathTagTraits<float> { static constexpr vr::PropertyTypeTag_t tag = vr::k_unFloatPropertyTag; };
  template <> struct VrPathTagTraits<bool> { static constexpr vr::PropertyTypeTag_t tag = vr::k_unBoolPropertyTag; };

  /**
   * Helper to write a single property to a Path Batch.
   * @param pVRPaths The IVRPaths interface pointer.
   * @param hContainer The handle to the block or queue container.
   * @param pchPath The string path (e.g., "/server_time_ticks").
   * @param value The data to write.
   */
  template <typename T>
  void WritePathProperty(vr::IVRPaths* pVRPaths, vr::PropertyContainerHandle_t hContainer, const char* pchPath, T value) {
    if (!pVRPaths || hContainer == vr::k_ulInvalidPropertyContainer) return;

    vr::PathHandle_t pathHandle;
    if (pVRPaths->StringToHandle(&pathHandle, pchPath) == vr::TrackedProp_Success) {
      vr::PathWrite_t batch = {};
      batch.ulPath = pathHandle;
      batch.pvBuffer = &value;
      batch.unBufferSize = sizeof(T);
      batch.unTag = VrPathTagTraits<T>::tag;
      batch.writeType = vr::PropertyWrite_Set; // Ensure we are setting the value

      pVRPaths->WritePathBatch(hContainer, &batch, 1);
    }
  }
}

namespace vr_internal {

  // ...

}
