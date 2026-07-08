// Copy into SimWorld Source (see INSTALL_NATIVE.md).
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "NavQueryService.generated.h"

class UNavModifierComponent;

USTRUCT()
struct FNavPlanningObstacle
{
	GENERATED_BODY()

	UPROPERTY()
	FString Id;

	UPROPERTY()
	float Cx = 0.0f;

	UPROPERTY()
	float Cy = 0.0f;

	UPROPERTY()
	float HalfX = 0.0f;

	UPROPERTY()
	float HalfY = 0.0f;
};

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

	/**
	 * Find path, resample along the NavMesh corridor, and reject when center-to-AABB
	 * clearance falls below MinCenterClearanceCm on any sample.
	 */
	UFUNCTION(BlueprintCallable, Category = "NavQuery")
	FString NavFindPathValidated(
		float StartX,
		float StartY,
		float StartZ,
		float EndX,
		float EndY,
		float EndZ,
		float AgentRadiusCm,
		float MinCenterClearanceCm,
		float ResampleSpacingCm);

	/** Register unpadded prop AABB used for post-plan clearance validation [cm]. */
	UFUNCTION(BlueprintCallable, Category = "NavQuery")
	FString NavRegisterPlanningObstacle(
		const FString& ObstacleId,
		float CenterX,
		float CenterY,
		float HalfExtentX,
		float HalfExtentY);

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

	/**
	 * Rebuild only NavMesh tiles overlapping a world-space AABB [cm].
	 * Does not mark the full NavMesh dirty (use NavRebuild for initial/static setup).
	 */
	UFUNCTION(BlueprintCallable, Category = "NavQuery")
	FString NavRebuildDirtyRegion(
		float MinX,
		float MinY,
		float MinZ,
		float MaxX,
		float MaxY,
		float MaxZ,
		float MarginCm);

protected:
	/** Half-extent [cm] used when projecting points onto NavMesh. */
	UPROPERTY(EditAnywhere, Category = "NavQuery")
	float ProjectExtentCm = 30.0f;

	/** Default agent radius [cm] for NavFindPath (center-to-surface planning clearance). */
	UPROPERTY(EditAnywhere, Category = "NavQuery")
	float DefaultAgentRadiusCm = 15.0f;

	/** Expand NavRebuild dirty bounds by this margin [cm] beyond modifier boxes. */
	UPROPERTY(EditAnywhere, Category = "NavQuery")
	float RebuildDirtyMarginCm = 200.0f;

	/** Segment sampling step when validating clearance along path chords [cm]. */
	UPROPERTY(EditAnywhere, Category = "NavQuery")
	float ClearanceSegmentSampleCm = 16.0f;

private:
	UPROPERTY()
	TMap<FString, TObjectPtr<AActor>> BoxObstacles;

	UPROPERTY()
	TArray<FNavPlanningObstacle> PlanningObstacles;

	FString NavFindPathInternal(
		float StartX,
		float StartY,
		float StartZ,
		float EndX,
		float EndY,
		float EndZ,
		float AgentRadiusCm,
		float MinCenterClearanceCm,
		float ResampleSpacingCm,
		bool bValidateClearance,
		bool bResampleOnNavPath);

	AActor* FindActorByNameOrLabel(const FString& ActorName) const;
	AActor* GetOrCreateBoxObstacle(const FString& ObstacleId);
};
