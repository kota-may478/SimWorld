// Copy into SimWorld Source (see INSTALL_NATIVE.md).
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "NavQueryService.generated.h"

class ANavModifierVolume;

UCLASS(Blueprintable)
class SIMWORLD_API ANavQueryService : public AActor
{
	GENERATED_BODY()

public:
	ANavQueryService();

	/** Project world point [cm] onto the default NavMesh. Returns JSON. */
	UFUNCTION(BlueprintCallable, Category = "NavQuery")
	FString NavProjectPoint(float X, float Y, float Z);

	/** Find path on NavMesh between two world points [cm]. Uses DefaultAgentRadiusCm. */
	UFUNCTION(BlueprintCallable, Category = "NavQuery")
	FString NavFindPath(
		float StartX,
		float StartY,
		float StartZ,
		float EndX,
		float EndY,
		float EndZ);

	/** Find path with explicit agent radius [cm] (center-to-nav-boundary clearance). */
	UFUNCTION(BlueprintCallable, Category = "NavQuery")
	FString NavFindPathWithRadius(
		float StartX,
		float StartY,
		float StartZ,
		float EndX,
		float EndY,
		float EndZ,
		float AgentRadiusCm);

	/** Whether a NavMesh path exists between two world points [cm]. Returns JSON. */
	UFUNCTION(BlueprintCallable, Category = "NavQuery")
	FString NavIsReachable(
		float StartX,
		float StartY,
		float StartZ,
		float EndX,
		float EndY,
		float EndZ);

	/** Axis-aligned actor bounds [cm]. Returns JSON {ok, cx, cy, cz, half_x, half_y, half_z}. */
	UFUNCTION(BlueprintCallable, Category = "NavQuery")
	FString GetActorBoundsJson(const FString& ActorName);

	/** Register or update a box nav obstacle (NavModifierVolume) by string id. Extents are half-sizes [cm]. */
	UFUNCTION(BlueprintCallable, Category = "NavQuery")
	FString NavRegisterBoxObstacle(
		const FString& ObstacleId,
		float CenterX,
		float CenterY,
		float CenterZ,
		float HalfExtentX,
		float HalfExtentY,
		float HalfExtentZ);

	/** Remove all runtime box obstacles created by this service. */
	UFUNCTION(BlueprintCallable, Category = "NavQuery")
	FString NavClearBoxObstacles();

	/** Rebuild navigation mesh after obstacle registration. Returns JSON. */
	UFUNCTION(BlueprintCallable, Category = "NavQuery")
	FString NavRebuild();

protected:
	/** Half-extent [cm] used when projecting points onto NavMesh. */
	UPROPERTY(EditAnywhere, Category = "NavQuery")
	float ProjectExtentCm = 30.0f;

	/** Default agent radius [cm] for NavFindPath (center-to-surface planning clearance). */
	UPROPERTY(EditAnywhere, Category = "NavQuery")
	float DefaultAgentRadiusCm = 100.0f;

private:
	UPROPERTY()
	TMap<FString, TObjectPtr<ANavModifierVolume>> BoxObstacles;

	FString NavFindPathInternal(
		float StartX,
		float StartY,
		float StartZ,
		float EndX,
		float EndY,
		float EndZ,
		float AgentRadiusCm);

	AActor* FindActorByNameOrLabel(const FString& ActorName) const;
	ANavModifierVolume* GetOrCreateBoxObstacle(const FString& ObstacleId);
};
