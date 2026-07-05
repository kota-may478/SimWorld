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

	/** Default forward speed [cm/s]. 5 km/h — matches metrics.py SPEED_LIMIT_CM_S. */
	UPROPERTY(EditAnywhere, Category = "NavMove")
	float RobotSpeedCmPerSec = 500000.0f / 3600.0f;

	UPROPERTY(EditAnywhere, Category = "NavMove")
	float MaxMoveCmPerStep = 25.0f;

	UPROPERTY(EditAnywhere, Category = "NavMove")
	float MaxTurnDegPerStep = 22.0f;

	/** Rotate before move only when heading error exceeds this [deg]. */
	UPROPERTY(EditAnywhere, Category = "NavMove")
	float RotateThresholdDeg = 45.0f;

	/** Move without pre-rotate when heading error is at or below this [deg]. */
	UPROPERTY(EditAnywhere, Category = "NavMove")
	float MoveHeadingToleranceDeg = 45.0f;

	/** Prefer polyline segment bearing when robot→WP bearing differs by less than this [deg]. */
	UPROPERTY(EditAnywhere, Category = "NavMove")
	float PathSegmentYawToleranceDeg = 15.0f;

	/** Bearing change below this [deg] is treated as a straight segment (no micro-rotate). */
	UPROPERTY(EditAnywhere, Category = "NavMove")
	float CornerYawDeg = 15.0f;

	UPROPERTY(EditAnywhere, Category = "NavMove")
	float WaypointReachToleranceCm = 12.0f;

	UPROPERTY(EditAnywhere, Category = "NavMove")
	float NavProjectExtentCm = 15.0f;

	UPROPERTY(EditAnywhere, Category = "NavMove")
	float DefaultAgentRadiusCm = 180.0f;

	UPROPERTY(EditAnywhere, Category = "NavMove")
	float StuckMoveThresholdCm = 8.0f;

	UPROPERTY(EditAnywhere, Category = "NavMove")
	int32 StuckUnchangedCycles = 3;

	/** Minimum Duration [s] for move / direct-yaw rotate commands (control-period floor). */
	UPROPERTY(EditAnywhere, Category = "NavMove")
	float BpCommandMinDurationSec = 0.06f;

	/** When true, apply yaw on the pawn directly (Rotate_Angle BP is broken in some builds). */
	UPROPERTY(EditAnywhere, Category = "NavMove")
	bool bUseDirectYawRotation = true;

	/** When true, translate the pawn directly (Move_Speed ProcessEvent may not move). */
	UPROPERTY(EditAnywhere, Category = "NavMove")
	bool bUseDirectTranslation = true;

	/** When true, snap pawn XY onto navigable mesh after motion (see bSnapAfterMove). */
	UPROPERTY(EditAnywhere, Category = "NavMove")
	bool bSnapPawnToNavMesh = true;

	/** Snap after each move command (off reduces fence-adjacent lateral pulls). */
	UPROPERTY(EditAnywhere, Category = "NavMove")
	bool bSnapAfterMove = false;

	/** Skip nav snap when lateral correction would exceed this [cm]. */
	UPROPERTY(EditAnywhere, Category = "NavMove")
	float MaxLateralSnapCm = 12.0f;

	/** Retry ProjectPointToNavigation with this extent [cm] when the default fails. */
	UPROPERTY(EditAnywhere, Category = "NavMove")
	float NavProjectRetryExtentCm = 60.0f;

	/** Extra wait [s] after BP Move_Speed / NavExecRotate timer commands. */
	UPROPERTY(EditAnywhere, Category = "NavMove")
	float BpMotionGraceSec = 0.15f;

	/** Minimum Duration [s] passed to NavExecRotate when bUseDirectYawRotation is false. */
	UPROPERTY(EditAnywhere, Category = "NavMove")
	float BpRotateMinDurationSec = 0.5f;

	/** When true, skip stuck detection while using BP async move (not direct translation). */
	UPROPERTY(EditAnywhere, Category = "NavMove")
	bool bSkipStuckCheckForBpMotion = true;

	/** Max extra wait [s] after min BP command time before failing with stuck/rotate_stuck. */
	UPROPERTY(EditAnywhere, Category = "NavMove")
	float BpCommandMaxWaitSec = 2.0f;

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
	float CommandIssueWorldTime = 0.0f;
	float CommandMaxWaitSec = 0.0f;
	float CommandStartYawDeg = 0.0f;
	FVector CommandStartLocation = FVector::ZeroVector;
	FVector LastProgressLocation = FVector::ZeroVector;
	int32 UnchangedCommandCycles = 0;
	bool bHasLastProgressLocation = false;
	bool bTickFollowPathRunning = false;
	FTimerHandle FollowTimerHandle;
	float LockedSegmentYawDeg = 0.0f;
	bool bHasLockedSegmentYaw = false;
	int32 SegmentLockWaypointIndex = -1;

	bool ParsePathJson(const FString& PathJson, TArray<FVector>& OutPoints) const;
	bool BuildPathToGoal(const FVector& GoalWorld);
	void BeginFollowPath(int32 RequestId);
	void TickFollowPath();
	void IssueNextMotionCommand();
	void ResetSegmentYawLock();
	void UpdateLockedSegmentYaw(int32 WaypointIndex, const FVector& Loc);
	bool IsCornerWaypoint(int32 WaypointIndex) const;
	bool InvokePawnFloatFloatInt(
		APawn* InPawn,
		FName FunctionName,
		float FirstFloat,
		float SecondFloat,
		int32 IntArg) const;
	bool ApplyDirectYawDelta(APawn* InPawn, float SignedAngleDeg) const;
	bool SnapPawnToNavMesh(APawn* InPawn) const;
	bool ApplyDirectMoveToward(APawn* InPawn, const FVector& TargetWorld, float MoveCm) const;
	bool ApplyDirectMoveAlongHeading(APawn* InPawn, float MoveCm, float HeadingYawDeg) const;
	bool CallPawnMoveSpeed(
		float Speed,
		float Duration,
		int32 Direction,
		const FVector& MoveTargetWorld) const;
	bool CallPawnMoveAlongHeading(float Speed, float Duration, float HeadingYawDeg) const;
	bool CallPawnRotate(float Duration, float SignedAngleDeg) const;
	void StartFollowTimer();
	void StopFollowTimer();
	float Dist2D(const FVector& A, const FVector& B) const;
	float YawToTargetDeg(const FVector& From, const FVector& To) const;
	float NormalizeAngleDeg(float Angle) const;
	FString StatusJson() const;
	void MarkFailed(const FString& Reason);
	void BeginActiveCommand(ECommandKind Kind, float Duration);
	float ComputeCommandWaitSec(ECommandKind Kind, float Duration) const;
	bool ShouldSkipStuckCheckForCommand(ECommandKind Kind) const;
	bool ShouldWaitForBpProgress(ECommandKind Kind) const;
	bool HasBpCommandProgress(const APawn* InPawn, ECommandKind Kind) const;
};
