// Copy into SimWorld Source (see NAVMESH_PHASE5_UE_SETUP.md).
#include "SpotDogNavController.h"

#include "AI/Navigation/NavigationTypes.h"
#include "Dom/JsonObject.h"
#include "GameFramework/Pawn.h"
#include "NavigationPath.h"
#include "NavigationSystem.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Misc/OutputDeviceNull.h"
#include "UObject/UnrealType.h"

ASpotDogNavController::ASpotDogNavController()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;
}

void ASpotDogNavController::BeginPlay()
{
	Super::BeginPlay();
	SetActorTickEnabled(true);
	if (!IsValid(GetPawn()))
	{
		UE_LOG(LogTemp, Warning, TEXT("SpotDogNavController BeginPlay: no pawn possessed"));
	}
}

void ASpotDogNavController::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	StopFollowTimer();
	Super::EndPlay(EndPlayReason);
}

void ASpotDogNavController::StartFollowTimer()
{
	// Follow loop is driven by AAIController::Tick only. Do not register a second
	// timer callback — duplicate TickFollowPath calls interrupt BP Move_Speed timers.
	SetActorTickEnabled(true);
}

void ASpotDogNavController::StopFollowTimer()
{
	if (UWorld* World = GetWorld())
	{
		World->GetTimerManager().ClearTimer(FollowTimerHandle);
	}
	FollowTimerHandle.Invalidate();
}

static UNavigationSystemV1* GetNavSys(UWorld* World)
{
	return World ? UNavigationSystemV1::GetCurrent(World) : nullptr;
}

static bool ProjectToNav(
	UNavigationSystemV1* NavSys,
	const FVector& QueryPoint,
	float ExtentCm,
	FNavLocation& OutNavLoc)
{
	if (!NavSys)
	{
		return false;
	}
	const FVector Extent(ExtentCm, ExtentCm, ExtentCm);
	return NavSys->ProjectPointToNavigation(QueryPoint, OutNavLoc, Extent);
}

float ASpotDogNavController::Dist2D(const FVector& A, const FVector& B) const
{
	return FVector::Dist2D(A, B);
}

float ASpotDogNavController::NormalizeAngleDeg(float Angle) const
{
	while (Angle > 180.0f)
	{
		Angle -= 360.0f;
	}
	while (Angle < -180.0f)
	{
		Angle += 360.0f;
	}
	return Angle;
}

float ASpotDogNavController::YawToTargetDeg(const FVector& From, const FVector& To) const
{
	const FVector Delta = To - From;
	return FMath::RadiansToDegrees(FMath::Atan2(Delta.Y, Delta.X));
}

bool ASpotDogNavController::InvokePawnFloatFloatInt(
	APawn* InPawn,
	FName FunctionName,
	float FirstFloat,
	float SecondFloat,
	int32 IntArg) const
{
	if (!IsValid(InPawn))
	{
		return false;
	}
	if (!InPawn->FindFunction(FunctionName))
	{
		return false;
	}

	// Match UnrealCV vbp: CallFunctionByNameWithArguments parses args in BP pin order.
	const FString Cmd = FString::Printf(
		TEXT("%s %f %f %d"),
		*FunctionName.ToString(),
		FirstFloat,
		SecondFloat,
		IntArg);
	FOutputDeviceNull NullAr;
	return InPawn->CallFunctionByNameWithArguments(*Cmd, NullAr, nullptr, true);
}

bool ASpotDogNavController::ApplyDirectYawDelta(APawn* InPawn, float SignedAngleDeg) const
{
	if (!IsValid(InPawn))
	{
		return false;
	}
	FRotator Rot = InPawn->GetActorRotation();
	Rot.Yaw = NormalizeAngleDeg(Rot.Yaw + SignedAngleDeg);
	InPawn->SetActorRotation(Rot);
	SnapPawnToNavMesh(InPawn);
	return true;
}

bool ASpotDogNavController::SnapPawnToNavMesh(APawn* InPawn) const
{
	if (!bSnapPawnToNavMesh || !IsValid(InPawn))
	{
		return false;
	}

	UWorld* World = GetWorld();
	UNavigationSystemV1* NavSys = GetNavSys(World);
	if (!NavSys)
	{
		return false;
	}

	const FVector Query = InPawn->GetActorLocation();
	FNavLocation NavLoc;
	if (!ProjectToNav(NavSys, Query, NavProjectExtentCm, NavLoc))
	{
		if (!ProjectToNav(NavSys, Query, NavProjectRetryExtentCm, NavLoc))
		{
			return false;
		}
	}

	const FVector Current = InPawn->GetActorLocation();
	const FVector Snapped(
		NavLoc.Location.X,
		NavLoc.Location.Y,
		Current.Z);
	InPawn->SetActorLocation(
		Snapped,
		false,
		nullptr,
		ETeleportType::TeleportPhysics);
	return true;
}

bool ASpotDogNavController::ApplyDirectMoveToward(
	APawn* InPawn,
	const FVector& TargetWorld,
	float MoveCm) const
{
	if (!IsValid(InPawn) || MoveCm <= KINDA_SMALL_NUMBER)
	{
		return false;
	}

	const FVector Loc = InPawn->GetActorLocation();
	FVector Delta = TargetWorld - Loc;
	Delta.Z = 0.0f;
	const float Dist = Delta.Size();
	if (Dist <= KINDA_SMALL_NUMBER)
	{
		return SnapPawnToNavMesh(InPawn);
	}

	Delta = Delta.GetSafeNormal() * FMath::Min(MoveCm, Dist);
	InPawn->SetActorLocation(
		Loc + Delta,
		false,
		nullptr,
		ETeleportType::TeleportPhysics);
	return SnapPawnToNavMesh(InPawn);
}

bool ASpotDogNavController::CallPawnMoveSpeed(
	float Speed,
	float Duration,
	int32 Direction,
	const FVector& MoveTargetWorld) const
{
	APawn* ControlledPawn = GetPawn();
	if (!IsValid(ControlledPawn))
	{
		return false;
	}

	const float MoveCm = FMath::Max(0.0f, Speed * Duration);

	if (bUseDirectTranslation)
	{
		return ApplyDirectMoveToward(ControlledPawn, MoveTargetWorld, MoveCm);
	}

	static const FName Candidates[] = {
		FName(TEXT("NavExecMoveSpeed")),
		FName(TEXT("Move_Speed")),
		FName(TEXT("MoveSpeed")),
	};
	for (const FName Name : Candidates)
	{
		if (InvokePawnFloatFloatInt(ControlledPawn, Name, Speed, Duration, Direction))
		{
			return true;
		}
	}
	return ApplyDirectMoveToward(ControlledPawn, MoveTargetWorld, MoveCm);
}

bool ASpotDogNavController::CallPawnRotate(
	float Duration,
	float SignedAngleDeg) const
{
	APawn* ControlledPawn = GetPawn();
	if (!IsValid(ControlledPawn))
	{
		return false;
	}

	if (bUseDirectYawRotation)
	{
		return ApplyDirectYawDelta(ControlledPawn, SignedAngleDeg);
	}

	const float AbsAngle = FMath::Abs(SignedAngleDeg);
	const int32 Clockwise = (SignedAngleDeg < 0.0f) ? 1 : -1;
	static const FName Candidates[] = {
		FName(TEXT("NavExecRotate")),
		FName(TEXT("Rotate_Angle")),
		FName(TEXT("Rotate")),
	};
	for (const FName Name : Candidates)
	{
		if (InvokePawnFloatFloatInt(ControlledPawn, Name, Duration, AbsAngle, Clockwise))
		{
			return true;
		}
	}
	return ApplyDirectYawDelta(ControlledPawn, SignedAngleDeg);
}

FString ASpotDogNavController::StatusJson() const
{
	const TCHAR* StatusStr = TEXT("idle");
	switch (MoveStatus)
	{
	case ESpotDogNavMoveStatus::Moving:
		StatusStr = TEXT("moving");
		break;
	case ESpotDogNavMoveStatus::Success:
		StatusStr = TEXT("success");
		break;
	case ESpotDogNavMoveStatus::Failed:
		StatusStr = TEXT("failed");
		break;
	default:
		break;
	}

	float DistRemaining = 0.0f;
	if (const APawn* ControlledPawn = GetPawn())
	{
		const FVector Loc = ControlledPawn->GetActorLocation();
		const FVector Target = (CurrentWaypointIndex < PathPoints.Num())
			? PathPoints[CurrentWaypointIndex]
			: GoalPoint;
		DistRemaining = Dist2D(Loc, Target);
	}

	return FString::Printf(
		TEXT("{\"status\":\"%s\",\"request_id\":%d,\"wp_index\":%d,\"wp_count\":%d,"
			 "\"dist_remaining_cm\":%.2f,\"acceptance_radius_cm\":%.2f}"),
		StatusStr,
		ActiveRequestId,
		CurrentWaypointIndex,
		PathPoints.Num(),
		DistRemaining,
		AcceptanceRadiusCm);
}

void ASpotDogNavController::MarkFailed(const FString& Reason)
{
	MoveStatus = ESpotDogNavMoveStatus::Failed;
	ActiveCommand = ECommandKind::None;
	StopFollowTimer();
	UE_LOG(LogTemp, Warning, TEXT("SpotDogNavController failed: %s"), *Reason);
}

float ASpotDogNavController::ComputeCommandWaitSec(
	ECommandKind Kind,
	float Duration) const
{
	float WaitSec = Duration;
	if (Kind == ECommandKind::Rotate && !bUseDirectYawRotation)
	{
		WaitSec += BpMotionGraceSec;
	}
	else if (Kind == ECommandKind::Move && !bUseDirectTranslation)
	{
		WaitSec += BpMotionGraceSec;
	}
	return WaitSec;
}

bool ASpotDogNavController::ShouldSkipStuckCheckForCommand(ECommandKind Kind) const
{
	if (!bSkipStuckCheckForBpMotion)
	{
		return false;
	}
	return Kind == ECommandKind::Move && !bUseDirectTranslation;
}

bool ASpotDogNavController::ShouldWaitForBpProgress(ECommandKind Kind) const
{
	if (Kind == ECommandKind::Rotate)
	{
		return !bUseDirectYawRotation;
	}
	if (Kind == ECommandKind::Move)
	{
		return !bUseDirectTranslation;
	}
	return false;
}

bool ASpotDogNavController::HasBpCommandProgress(
	const APawn* InPawn,
	ECommandKind Kind) const
{
	if (!IsValid(InPawn))
	{
		return false;
	}

	if (Kind == ECommandKind::Rotate)
	{
		const float YawDelta = FMath::Abs(
			NormalizeAngleDeg(InPawn->GetActorRotation().Yaw - CommandStartYawDeg));
		return YawDelta >= 2.0f;
	}

	if (Kind == ECommandKind::Move)
	{
		return Dist2D(InPawn->GetActorLocation(), CommandStartLocation)
			>= StuckMoveThresholdCm;
	}

	return false;
}

void ASpotDogNavController::BeginActiveCommand(ECommandKind Kind, float Duration)
{
	APawn* ControlledPawn = GetPawn();
	const UWorld* World = GetWorld();
	const float Now = World ? World->GetTimeSeconds() : 0.0f;

	UnchangedCommandCycles = 0;
	bHasLastProgressLocation = false;
	ActiveCommand = Kind;
	CommandIssueWorldTime = Now;
	CommandEndWorldTime = Now + ComputeCommandWaitSec(Kind, Duration);
	CommandMaxWaitSec = Duration + BpMotionGraceSec + BpCommandMaxWaitSec;

	if (IsValid(ControlledPawn))
	{
		CommandStartLocation = ControlledPawn->GetActorLocation();
		CommandStartYawDeg = ControlledPawn->GetActorRotation().Yaw;
	}
}

bool ASpotDogNavController::ParsePathJson(
	const FString& PathJson,
	TArray<FVector>& OutPoints) const
{
	OutPoints.Reset();
	TSharedPtr<FJsonObject> Root;
	const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(PathJson);
	if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid())
	{
		return false;
	}

	const TArray<TSharedPtr<FJsonValue>>* Points = nullptr;
	if (!Root->TryGetArrayField(TEXT("points"), Points) || !Points)
	{
		return false;
	}

	for (const TSharedPtr<FJsonValue>& Value : *Points)
	{
		const TSharedPtr<FJsonObject> PointObj = Value->AsObject();
		if (!PointObj.IsValid())
		{
			continue;
		}
		double X = 0.0;
		double Y = 0.0;
		double Z = 0.0;
		PointObj->TryGetNumberField(TEXT("x"), X);
		PointObj->TryGetNumberField(TEXT("y"), Y);
		PointObj->TryGetNumberField(TEXT("z"), Z);
		OutPoints.Add(FVector(static_cast<float>(X), static_cast<float>(Y), static_cast<float>(Z)));
	}
	return OutPoints.Num() > 0;
}

bool ASpotDogNavController::BuildPathToGoal(const FVector& GoalWorld)
{
	PathPoints.Reset();
	GoalPoint = GoalWorld;

	APawn* ControlledPawn = GetPawn();
	UWorld* World = GetWorld();
	if (!IsValid(ControlledPawn) || !World)
	{
		return false;
	}

	UNavigationSystemV1* NavSys = GetNavSys(World);
	if (!NavSys)
	{
		return false;
	}

	FNavLocation StartNav;
	FNavLocation EndNav;
	const FVector StartQuery = ControlledPawn->GetActorLocation();
	if (!ProjectToNav(NavSys, StartQuery, NavProjectExtentCm, StartNav)
		&& !ProjectToNav(NavSys, StartQuery, NavProjectRetryExtentCm, StartNav))
	{
		return false;
	}
	if (!ProjectToNav(NavSys, GoalWorld, NavProjectExtentCm, EndNav)
		&& !ProjectToNav(NavSys, GoalWorld, NavProjectRetryExtentCm, EndNav))
	{
		return false;
	}

	const ANavigationData* NavData = NavSys->GetDefaultNavDataInstance(
		FNavigationSystem::DontCreate);
	if (!NavData)
	{
		return false;
	}

	FNavAgentProperties AgentProps;
	AgentProps.AgentRadius = FMath::Max(1.0f, DefaultAgentRadiusCm);
	AgentProps.AgentHeight = 200.0f;

	const FPathFindingQuery Query(
		this,
		*NavData,
		StartNav.Location,
		EndNav.Location);
	const FPathFindingResult Result = NavSys->FindPathSync(AgentProps, Query);
	if (!Result.IsSuccessful() || !Result.Path.IsValid())
	{
		return false;
	}

	const TArray<FNavPathPoint>& NavPoints = Result.Path->GetPathPoints();
	for (const FNavPathPoint& Point : NavPoints)
	{
		PathPoints.Add(Point.Location);
	}
	GoalPoint = EndNav.Location;
	return PathPoints.Num() > 0;
}

void ASpotDogNavController::BeginFollowPath(int32 RequestId)
{
	ActiveRequestId = RequestId;
	CurrentWaypointIndex = 0;
	MoveStatus = ESpotDogNavMoveStatus::Moving;
	ActiveCommand = ECommandKind::None;
	CommandEndWorldTime = 0.0f;
	CommandIssueWorldTime = 0.0f;
	CommandMaxWaitSec = 0.0f;
	UnchangedCommandCycles = 0;
	bHasLastProgressLocation = false;
	bTickFollowPathRunning = false;
	if (APawn* ControlledPawn = GetPawn())
	{
		SnapPawnToNavMesh(ControlledPawn);
	}
	StartFollowTimer();
}

FString ASpotDogNavController::NavMoveToGoal(
	float GoalX,
	float GoalY,
	float GoalZ,
	float InAcceptanceRadiusCm)
{
	if (!IsValid(GetPawn()))
	{
		return TEXT("{\"ok\":false,\"error\":\"no_pawn\"}");
	}

	AcceptanceRadiusCm = FMath::Max(10.0f, InAcceptanceRadiusCm);
	const FVector Goal(GoalX, GoalY, GoalZ);
	if (!BuildPathToGoal(Goal))
	{
		return TEXT("{\"ok\":false,\"error\":\"no_path\"}");
	}

	const int32 RequestId = ActiveRequestId + 1;
	BeginFollowPath(RequestId);
	IssueNextMotionCommand();
	return FString::Printf(
		TEXT("{\"ok\":true,\"request_id\":%d,\"wp_count\":%d}"),
		RequestId,
		PathPoints.Num());
}

FString ASpotDogNavController::NavFollowPathJson(const FString& PathJson)
{
	if (!IsValid(GetPawn()))
	{
		return TEXT("{\"ok\":false,\"error\":\"no_pawn\"}");
	}

	TArray<FVector> Parsed;
	if (!ParsePathJson(PathJson, Parsed))
	{
		return TEXT("{\"ok\":false,\"error\":\"bad_path_json\"}");
	}

	PathPoints = Parsed;
	GoalPoint = Parsed.Last();
	const int32 RequestId = ActiveRequestId + 1;
	BeginFollowPath(RequestId);
	IssueNextMotionCommand();
	return FString::Printf(
		TEXT("{\"ok\":true,\"request_id\":%d,\"wp_count\":%d}"),
		RequestId,
		PathPoints.Num());
}

FString ASpotDogNavController::NavStopMove()
{
	PathPoints.Reset();
	CurrentWaypointIndex = 0;
	MoveStatus = ESpotDogNavMoveStatus::Idle;
	ActiveCommand = ECommandKind::None;
	CommandEndWorldTime = 0.0f;
	StopFollowTimer();
	return TEXT("{\"ok\":true}");
}

FString ASpotDogNavController::GetNavMoveStatusJson()
{
	return StatusJson();
}

void ASpotDogNavController::IssueNextMotionCommand()
{
	APawn* ControlledPawn = GetPawn();
	if (!IsValid(ControlledPawn) || MoveStatus != ESpotDogNavMoveStatus::Moving)
	{
		return;
	}

	const FVector Loc = ControlledPawn->GetActorLocation();
	const float YawDeg = ControlledPawn->GetActorRotation().Yaw;

	if (Dist2D(Loc, GoalPoint) <= AcceptanceRadiusCm)
	{
		MoveStatus = ESpotDogNavMoveStatus::Success;
		ActiveCommand = ECommandKind::None;
		StopFollowTimer();
		return;
	}

	while (CurrentWaypointIndex < PathPoints.Num()
		&& Dist2D(Loc, PathPoints[CurrentWaypointIndex]) <= WaypointReachToleranceCm)
	{
		CurrentWaypointIndex++;
	}

	const FVector Target = (CurrentWaypointIndex < PathPoints.Num())
		? PathPoints[CurrentWaypointIndex]
		: GoalPoint;

	const float DistanceCm = Dist2D(Loc, Target);
	if (DistanceCm < 1.0f)
	{
		CurrentWaypointIndex++;
		return;
	}

	const float TargetYaw = YawToTargetDeg(Loc, Target);
	const float AngleDiff = NormalizeAngleDeg(TargetYaw - YawDeg);
	const float SafeSpeed = FMath::Max(RobotSpeedCmPerSec, 1.0f);

	if (FMath::Abs(AngleDiff) > RotateThresholdDeg)
	{
		const float TurnDeg = FMath::Min(FMath::Abs(AngleDiff), MaxTurnDegPerStep);
		const float SignedTurnDeg = FMath::Sign(AngleDiff) * TurnDeg;
		const float Duration = bUseDirectYawRotation
			? FMath::Max(0.12f, TurnDeg / SafeSpeed)
			: FMath::Max(BpRotateMinDurationSec, TurnDeg / 30.0f);
		if (!CallPawnRotate(Duration, SignedTurnDeg))
		{
			MarkFailed(TEXT("rotate_vbp_missing"));
			return;
		}
		BeginActiveCommand(ECommandKind::Rotate, Duration);
		return;
	}

	const float MoveCm = FMath::Min(DistanceCm, MaxMoveCmPerStep);
	const float Duration = FMath::Max(0.12f, MoveCm / SafeSpeed);
	if (!CallPawnMoveSpeed(SafeSpeed, Duration, 0, Target))
	{
		MarkFailed(TEXT("move_vbp_missing"));
		return;
	}
	BeginActiveCommand(ECommandKind::Move, Duration);
}

void ASpotDogNavController::TickFollowPath()
{
	if (bTickFollowPathRunning)
	{
		return;
	}

	struct FTickFollowScope
	{
		bool& Flag;
		explicit FTickFollowScope(bool& InFlag) : Flag(InFlag) { Flag = true; }
		~FTickFollowScope() { Flag = false; }
	};
	FTickFollowScope ReentryScope(bTickFollowPathRunning);

	APawn* ControlledPawn = GetPawn();
	if (!IsValid(ControlledPawn))
	{
		MarkFailed(TEXT("pawn_lost"));
		return;
	}

	const UWorld* World = GetWorld();
	if (!World)
	{
		MarkFailed(TEXT("no_world"));
		return;
	}

	const float Now = World->GetTimeSeconds();
	if (ActiveCommand != ECommandKind::None)
	{
		if (Now < CommandEndWorldTime)
		{
			return;
		}

		if (ShouldWaitForBpProgress(ActiveCommand)
			&& !HasBpCommandProgress(ControlledPawn, ActiveCommand))
		{
			if (Now < CommandIssueWorldTime + CommandMaxWaitSec)
			{
				return;
			}
			MarkFailed(
				ActiveCommand == ECommandKind::Rotate
					? TEXT("rotate_stuck")
					: TEXT("stuck"));
			return;
		}
	}

	if (ActiveCommand != ECommandKind::None)
	{
		if (ActiveCommand == ECommandKind::Move)
		{
			SnapPawnToNavMesh(ControlledPawn);
			if (!ShouldSkipStuckCheckForCommand(ECommandKind::Move))
			{
				const FVector SnappedLoc = ControlledPawn->GetActorLocation();
				if (bHasLastProgressLocation
					&& Dist2D(SnappedLoc, LastProgressLocation) < StuckMoveThresholdCm)
				{
					UnchangedCommandCycles++;
					if (UnchangedCommandCycles >= StuckUnchangedCycles)
					{
						MarkFailed(TEXT("stuck"));
						return;
					}
				}
				else
				{
					UnchangedCommandCycles = 0;
				}
				LastProgressLocation = SnappedLoc;
				bHasLastProgressLocation = true;
			}
			else
			{
				UnchangedCommandCycles = 0;
			}
		}
		ActiveCommand = ECommandKind::None;
	}

	if (Dist2D(ControlledPawn->GetActorLocation(), GoalPoint) <= AcceptanceRadiusCm)
	{
		MoveStatus = ESpotDogNavMoveStatus::Success;
		StopFollowTimer();
		return;
	}

	IssueNextMotionCommand();
}

void ASpotDogNavController::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (MoveStatus == ESpotDogNavMoveStatus::Moving)
	{
		TickFollowPath();
	}
}
