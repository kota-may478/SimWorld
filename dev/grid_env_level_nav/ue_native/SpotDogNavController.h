// Copy into SimWorld Source (see NAVMESH_PHASE5_UE_SETUP.md).
#pragma once

#include "CoreMinimal.h"
#include "AIController.h"
#include "TimerManager.h"
#include "SpotDogNavController.generated.h"

UENUM(BlueprintType)
enum class ESpotDogNavMoveStatus : uint8
{
	Idle UMETA(DisplayName = "Idle"),
	Moving UMETA(DisplayName = "Moving"),
	Success UMETA(DisplayName = "Success"),
	Failed UMETA(DisplayName = "Failed"),
};

UCLASS(Blueprintable)
class SIMWORLD_API ASpotDogNavController : public AAIController
{
	GENERATED_BODY()

public:
	ASpotDogNavController();

	/** Move to a world goal [cm] using internal NavFindPath + UE-side open-loop follow. */
	UFUNCTION(BlueprintCallable, Category = "NavMove")
	FString NavMoveToGoal(float GoalX, float GoalY, float GoalZ, float AcceptanceRadiusCm);

	/** Follow a pre-planned path JSON: {"points":[{"x":..,"y":..,"z":..}, ...]}. */
	UFUNCTION(BlueprintCallable, Category = "NavMove")
	FString NavFollowPathJson(const FString& PathJson);

	/** Stop the current follow request. */
	UFUNCTION(BlueprintCallable, Category = "NavMove")
	FString NavStopMove();

	/** Poll follow status JSON for Python. */
	UFUNCTION(BlueprintCallable, Category = "NavMove")
	FString GetNavMoveStatusJson();

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

	/** Default forward speed [cm/s] (matches site_transport navmesh profile). */
	UPROPERTY(EditAnywhere, Category = "NavMove")
	float RobotSpeedCmPerSec = 180.0f;

	UPROPERTY(EditAnywhere, Category = "NavMove")
	float MaxMoveCmPerStep = 180.0f;

	UPROPERTY(EditAnywhere, Category = "NavMove")
	float MaxTurnDegPerStep = 22.0f;

	UPROPERTY(EditAnywhere, Category = "NavMove")
	float RotateThresholdDeg = 6.0f;

	UPROPERTY(EditAnywhere, Category = "NavMove")
	float WaypointReachToleranceCm = 80.0f;

	UPROPERTY(EditAnywhere, Category = "NavMove")
	float NavProjectExtentCm = 30.0f;

	UPROPERTY(EditAnywhere, Category = "NavMove")
	float DefaultAgentRadiusCm = 100.0f;

	UPROPERTY(EditAnywhere, Category = "NavMove")
	float StuckMoveThresholdCm = 8.0f;

	UPROPERTY(EditAnywhere, Category = "NavMove")
	int32 StuckUnchangedCycles = 3;

	/** When true, apply yaw on the pawn directly (Rotate_Angle BP is broken in some builds). */
	UPROPERTY(EditAnywhere, Category = "NavMove")
	bool bUseDirectYawRotation = true;

	/** When true, translate the pawn directly (Move_Speed ProcessEvent may not move). */
	UPROPERTY(EditAnywhere, Category = "NavMove")
	bool bUseDirectTranslation = true;

	/** When true, snap pawn XY/Z onto navigable mesh after each motion step. */
	UPROPERTY(EditAnywhere, Category = "NavMove")
	bool bSnapPawnToNavMesh = true;

	/** Retry ProjectPointToNavigation with this extent [cm] when the default fails. */
	UPROPERTY(EditAnywhere, Category = "NavMove")
	float NavProjectRetryExtentCm = 120.0f;

private:
	enum class ECommandKind : uint8
	{
		None,
		Rotate,
		Move,
	};

	TArray<FVector> PathPoints;
	FVector GoalPoint = FVector::ZeroVector;
	int32 CurrentWaypointIndex = 0;
	int32 ActiveRequestId = 0;
	float AcceptanceRadiusCm = 130.0f;
	ESpotDogNavMoveStatus MoveStatus = ESpotDogNavMoveStatus::Idle;
	ECommandKind ActiveCommand = ECommandKind::None;
	float CommandEndWorldTime = 0.0f;
	FVector LastProgressLocation = FVector::ZeroVector;
	int32 UnchangedCommandCycles = 0;
	bool bHasLastProgressLocation = false;
	FTimerHandle FollowTimerHandle;

	bool ParsePathJson(const FString& PathJson, TArray<FVector>& OutPoints) const;
	bool BuildPathToGoal(const FVector& GoalWorld);
	void BeginFollowPath(int32 RequestId);
	void TickFollowPath();
	void IssueNextMotionCommand();
	bool InvokePawnFloatFloatInt(
		APawn* InPawn,
		FName FunctionName,
		float FirstFloat,
		float SecondFloat,
		int32 IntArg) const;
	bool ApplyDirectYawDelta(APawn* InPawn, float SignedAngleDeg) const;
	bool SnapPawnToNavMesh(APawn* InPawn) const;
	bool ApplyDirectMoveToward(APawn* InPawn, const FVector& TargetWorld, float MoveCm) const;
	bool CallPawnMoveSpeed(
		float Speed,
		float Duration,
		int32 Direction,
		const FVector& MoveTargetWorld) const;
	bool CallPawnRotate(float Duration, float SignedAngleDeg) const;
	void StartFollowTimer();
	void StopFollowTimer();
	float Dist2D(const FVector& A, const FVector& B) const;
	float YawToTargetDeg(const FVector& From, const FVector& To) const;
	float NormalizeAngleDeg(float Angle) const;
	FString StatusJson() const;
	void MarkFailed(const FString& Reason);
};
