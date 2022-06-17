// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

// UE4Carla - base color (for lighting)

#include "Carla.h"
#include "Carla/Sensor/MatCamera.h"

#include "Carla/Sensor/PixelReader.h"

FActorDefinition AMatCamera::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeCameraDefinition(TEXT("mat_map"));
}

AMatCamera::AMatCamera(
    const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
  AddPostProcessingMaterial(
      TEXT("Material'/Carla/PostProcessingMaterials/PhysicLensDistortion.PhysicLensDistortion'"));
  AddPostProcessingMaterial(
      TEXT("Material'/Carla/PostProcessingMaterials/matecolor.matecolor'"));
}

void AMatCamera::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaSeconds)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(AMatCamera::PostPhysTick);
  FPixelReader::SendPixelsInRenderThread(*this);
}
