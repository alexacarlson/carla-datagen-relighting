// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "Carla/Sensor/LidarIntensityCamera.h"

#include "Carla/Sensor/PixelReader.h"

FActorDefinition ALidarIntensityCamera::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeCameraDefinition(TEXT("lidar_reflectance"));
}

ALidarIntensityCamera::ALidarIntensityCamera(
    const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
  AddPostProcessingMaterial(
      TEXT("Material'/Carla/PostProcessingMaterials/LidarIntensity.LidarIntensity'"));
}

void ALidarIntensityCamera::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaSeconds)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ALidarIntensityCamera::PostPhysTick);
  FPixelReader::SendPixelsInRenderThread(*this);
}
