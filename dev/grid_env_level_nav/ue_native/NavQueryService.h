// Copy into SimWorld Source (see INSTALL_NATIVE.md).
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "NavQueryService.generated.h"

UCLASS(Blueprintable)
class SIMWORLD_API ANavQueryService : public AActor
{
	GENERATED_BODY()

public:
	ANavQueryService();

	/** Project world point [cm] onto the default NavMesh. Returns JSON. */
	UFUNCTION(BlueprintCallable, Category = "NavQuery")
	FString NavProjectPoint(float X, float Y, float Z);

	/** Find path on NavMesh between two world points [cm]. Returns JSON. */
	UFUNCTION(BlueprintCallable, Category = "NavQuery")
	FString NavFindPath(
		float StartX,
		float StartY,
		float StartZ,
		float EndX,
		float EndY,
		float EndZ);

	/** Whether a NavMesh path exists between two world points [cm]. Returns JSON. */
	UFUNCTION(BlueprintCallable, Category = "NavQuery")
	FString NavIsReachable(
		float StartX,
		float StartY,
		float StartZ,
		float EndX,
		float EndY,
		float EndZ);

protected:
	/** Half-extent [cm] used when projecting points onto NavMesh. */
	UPROPERTY(EditAnywhere, Category = "NavQuery")
	float ProjectExtentCm = 30.0f;
};
