// Optional native parent for BP_SemanticCollisionProbe (copy into SimWorld Source).
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SemanticCollisionProbe.generated.h"

UCLASS(Blueprintable)
class SIMWORLD_API ASemanticCollisionProbe : public AActor
{
	GENERATED_BODY()

public:
	ASemanticCollisionProbe();

	/** Sphere sweep at world point (cm). ``RadiusCm`` <= 0 uses ``ProbeRadiusCm``. */
	UFUNCTION(BlueprintCallable, Category = "SemanticProbe")
	FString ProbePointHit(float X, float Y, float Z, float RadiusCm = 0.f);

protected:
	/** Default when ``RadiusCm`` not passed (inscribed sphere in 0.3 m cube = 15 cm). */
	UPROPERTY(EditAnywhere, Category = "SemanticProbe")
	float ProbeRadiusCm = 15.0f;
};
