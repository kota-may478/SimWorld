// Copy into SimWorld Source (see INSTALL_NATIVE.md).
#include "NavQueryService.h"

#include "Components/SceneComponent.h"
#include "EngineUtils.h"
#include "NavAreas/NavArea_Obstacle.h"
#include "AI/Navigation/NavigationTypes.h"
#include "NavigationData.h"
#include "NavigationPath.h"
#include "NavigationSystem.h"
#include "NavModifierComponent.h"

ANavQueryService::ANavQueryService()
{
	PrimaryActorTick.bCanEverTick = false;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
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

static float CenterToAabbSurface2D(
	const FVector& Point,
	const FNavPlanningObstacle& Obstacle)
{
	const float Dx = FMath::Abs(Point.X - Obstacle.Cx) - Obstacle.HalfX;
	const float Dy = FMath::Abs(Point.Y - Obstacle.Cy) - Obstacle.HalfY;
	const float ClampedX = FMath::Max(0.0f, Dx);
	const float ClampedY = FMath::Max(0.0f, Dy);
	return FMath::Sqrt(ClampedX * ClampedX + ClampedY * ClampedY);
}

static float MinCenterClearanceAtPoint2D(
	const FVector& Point,
	const TArray<FNavPlanningObstacle>& Obstacles,
	FString* OutWorstId = nullptr)
{
	float Best = TNumericLimits<float>::Max();
	FString WorstId;
	for (const FNavPlanningObstacle& Obstacle : Obstacles)
	{
		const float Dist = CenterToAabbSurface2D(Point, Obstacle);
		if (Dist < Best)
		{
			Best = Dist;
			WorstId = Obstacle.Id;
		}
	}
	if (OutWorstId != nullptr)
	{
		*OutWorstId = WorstId;
	}
	return Best;
}

static float MinCenterClearanceOnSegment2D(
	const FVector& Start,
	const FVector& End,
	const TArray<FNavPlanningObstacle>& Obstacles,
	float SampleSpacingCm,
	UNavigationSystemV1* NavSys,
	float ProjectExtentCm,
	FString* OutWorstId = nullptr)
{
	const float SegLen = FVector::Dist2D(Start, End);
	if (SegLen < KINDA_SMALL_NUMBER)
	{
		return MinCenterClearanceAtPoint2D(Start, Obstacles, OutWorstId);
	}

	const float StepCm = FMath::Max(1.0f, SampleSpacingCm);
	const int32 Samples = FMath::Max(2, FMath::CeilToInt(SegLen / StepCm));
	float Best = TNumericLimits<float>::Max();
	FString WorstId;
	for (int32 Index = 0; Index <= Samples; ++Index)
	{
		const float T = static_cast<float>(Index) / static_cast<float>(Samples);
		FVector Sample(
			Start.X + (End.X - Start.X) * T,
			Start.Y + (End.Y - Start.Y) * T,
			Start.Z + (End.Z - Start.Z) * T);
		if (NavSys)
		{
			FNavLocation Projected;
			if (ProjectToNav(NavSys, Sample, ProjectExtentCm, Projected))
			{
				Sample = Projected.Location;
			}
		}
		FString PointWorstId;
		const float Dist = MinCenterClearanceAtPoint2D(Sample, Obstacles, &PointWorstId);
		if (Dist < Best)
		{
			Best = Dist;
			WorstId = PointWorstId;
		}
	}
	if (OutWorstId != nullptr)
	{
		*OutWorstId = WorstId;
	}
	return Best;
}

static bool ValidatePathClearance(
	const TArray<FVector>& Points,
	const TArray<FNavPlanningObstacle>& Obstacles,
	float MinCenterClearanceCm,
	float SegmentSampleCm,
	UNavigationSystemV1* NavSys,
	float ProjectExtentCm,
	float& OutMinClearanceCm,
	FString& OutWorstObstacleId,
	int32& OutWorstPointIndex)
{
	OutMinClearanceCm = TNumericLimits<float>::Max();
	OutWorstObstacleId.Reset();
	OutWorstPointIndex = INDEX_NONE;

	if (Obstacles.Num() == 0 || MinCenterClearanceCm <= 0.0f)
	{
		return true;
	}

	const int32 LastIndex = Points.Num() - 1;
	for (int32 Index = 0; Index < Points.Num(); ++Index)
	{
		// Start and goal may sit near exempt props (humanoid, material); transit WPs only.
		if (Index == 0 || Index == LastIndex)
		{
			continue;
		}
		FString WorstId;
		const float Dist = MinCenterClearanceAtPoint2D(Points[Index], Obstacles, &WorstId);
		if (Dist < OutMinClearanceCm)
		{
			OutMinClearanceCm = Dist;
			OutWorstObstacleId = WorstId;
			OutWorstPointIndex = Index;
		}
	}

	// Planning adoption checks transit waypoint positions only (start/goal exempt).

	return OutMinClearanceCm >= MinCenterClearanceCm;
}

static FVector LocationAtDistanceAlongPolyline(
	const TArray<FNavPathPoint>& CornerPoints,
	const TArray<float>& CumulativeLength,
	float Distance)
{
	if (CornerPoints.Num() == 0)
	{
		return FVector::ZeroVector;
	}
	if (CornerPoints.Num() == 1 || Distance <= 0.0f)
	{
		return CornerPoints[0].Location;
	}

	const float ClampedDistance = FMath::Clamp(Distance, 0.0f, CumulativeLength.Last());
	int32 SegmentEnd = 1;
	while (SegmentEnd < CumulativeLength.Num()
		&& CumulativeLength[SegmentEnd] < ClampedDistance)
	{
		++SegmentEnd;
	}

	const float SegmentStartDist = CumulativeLength[SegmentEnd - 1];
	const float SegmentLen = CumulativeLength[SegmentEnd] - SegmentStartDist;
	const float Alpha = SegmentLen > KINDA_SMALL_NUMBER
		? (ClampedDistance - SegmentStartDist) / SegmentLen
		: 0.0f;
	return FMath::Lerp(
		CornerPoints[SegmentEnd - 1].Location,
		CornerPoints[SegmentEnd].Location,
		Alpha);
}

static TArray<FVector> ResampleNavigationPath(
	const FNavigationPath& Path,
	float SpacingCm,
	UNavigationSystemV1* NavSys,
	float ProjectExtentCm)
{
	TArray<FVector> Out;
	const TArray<FNavPathPoint>& CornerPoints = Path.GetPathPoints();
	if (CornerPoints.Num() == 0)
	{
		return Out;
	}
	if (CornerPoints.Num() == 1)
	{
		Out.Add(CornerPoints[0].Location);
		return Out;
	}

	TArray<float> CumulativeLength;
	CumulativeLength.Reserve(CornerPoints.Num());
	CumulativeLength.Add(0.0f);
	for (int32 Index = 1; Index < CornerPoints.Num(); ++Index)
	{
		const float SegmentLen = FVector::Dist(
			CornerPoints[Index - 1].Location,
			CornerPoints[Index].Location);
		CumulativeLength.Add(CumulativeLength.Last() + SegmentLen);
	}

	const float TotalLength = CumulativeLength.Last();
	if (TotalLength <= KINDA_SMALL_NUMBER)
	{
		Out.Add(CornerPoints[0].Location);
		return Out;
	}

	auto AddResampledPoint = [&](const FVector& Candidate)
	{
		FVector Loc = Candidate;
		if (NavSys)
		{
			FNavLocation Projected;
			if (ProjectToNav(NavSys, Candidate, ProjectExtentCm, Projected))
			{
				Loc = Projected.Location;
			}
		}
		if (Out.Num() == 0 || FVector::Dist2D(Out.Last(), Loc) > 1.0f)
		{
			Out.Add(Loc);
		}
	};

	const float Step = FMath::Max(10.0f, SpacingCm);
	for (float Distance = 0.0f; Distance <= TotalLength; Distance += Step)
	{
		AddResampledPoint(LocationAtDistanceAlongPolyline(
			CornerPoints,
			CumulativeLength,
			Distance));
	}

	AddResampledPoint(CornerPoints.Last().Location);
	return Out;
}

static FString PointsToJson(const TArray<FVector>& Points)
{
	FString PointsJson = TEXT("[");
	for (int32 Index = 0; Index < Points.Num(); ++Index)
	{
		const FVector& Loc = Points[Index];
		if (Index > 0)
		{
			PointsJson += TEXT(",");
		}
		PointsJson += FString::Printf(
			TEXT("{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}"),
			Loc.X,
			Loc.Y,
			Loc.Z);
	}
	PointsJson += TEXT("]");
	return PointsJson;
}

AActor* ANavQueryService::FindActorByNameOrLabel(const FString& ActorName) const
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return nullptr;
	}

	for (TActorIterator<AActor> It(World); It; ++It)
	{
		AActor* Actor = *It;
		if (!IsValid(Actor))
		{
			continue;
		}
		if (Actor->GetName() == ActorName || Actor->GetFName().ToString() == ActorName)
		{
			return Actor;
		}
#if WITH_EDITOR
		if (Actor->GetActorLabel() == ActorName)
		{
			return Actor;
		}
#endif
	}
	return nullptr;
}

FString ANavQueryService::NavProjectPoint(float X, float Y, float Z)
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return TEXT("{\"ok\":false,\"error\":\"no_world\"}");
	}

	UNavigationSystemV1* NavSys = GetNavSys(World);
	if (!NavSys)
	{
		return TEXT("{\"ok\":false,\"error\":\"no_navsys\"}");
	}

	FNavLocation NavLoc;
	const FVector QueryPoint(X, Y, Z);
	if (!ProjectToNav(NavSys, QueryPoint, ProjectExtentCm, NavLoc))
	{
		return TEXT("{\"ok\":false,\"error\":\"no_projection\"}");
	}

	const FVector& P = NavLoc.Location;
	return FString::Printf(
		TEXT("{\"ok\":true,\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}"),
		P.X,
		P.Y,
		P.Z);
}

FString ANavQueryService::NavFindPathInternal(
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
	bool bResampleOnNavPath)
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return TEXT("{\"ok\":false,\"error\":\"no_world\"}");
	}

	UNavigationSystemV1* NavSys = GetNavSys(World);
	if (!NavSys)
	{
		return TEXT("{\"ok\":false,\"error\":\"no_navsys\"}");
	}

	FNavLocation StartNav;
	FNavLocation EndNav;
	const FVector StartQuery(StartX, StartY, StartZ);
	const FVector EndQuery(EndX, EndY, EndZ);
	if (!ProjectToNav(NavSys, StartQuery, ProjectExtentCm, StartNav))
	{
		return TEXT("{\"ok\":false,\"error\":\"start_not_on_navmesh\"}");
	}
	if (!ProjectToNav(NavSys, EndQuery, ProjectExtentCm, EndNav))
	{
		return TEXT("{\"ok\":false,\"error\":\"end_not_on_navmesh\"}");
	}

	const ANavigationData* NavData = NavSys->GetDefaultNavDataInstance(
		FNavigationSystem::DontCreate);
	if (!NavData)
	{
		return TEXT("{\"ok\":false,\"error\":\"no_navdata\"}");
	}

	FNavAgentProperties AgentProps;
	AgentProps.AgentRadius = FMath::Max(1.0f, AgentRadiusCm);
	AgentProps.AgentHeight = 200.0f;

	const FPathFindingQuery Query(
		this,
		*NavData,
		StartNav.Location,
		EndNav.Location);
	FPathFindingQuery QueryWithAgent = Query;
	QueryWithAgent.SetNavAgentProperties(AgentProps);
	const FPathFindingResult Result = NavSys->FindPathSync(AgentProps, QueryWithAgent);
	if (!Result.IsSuccessful() || !Result.Path.IsValid())
	{
		return TEXT("{\"ok\":false,\"error\":\"no_path\"}");
	}

	const TArray<FNavPathPoint>& CornerPoints = Result.Path->GetPathPoints();
	TArray<FVector> OutputPoints;
	if (bResampleOnNavPath && ResampleSpacingCm > 0.0f)
	{
		OutputPoints = ResampleNavigationPath(
			*Result.Path,
			ResampleSpacingCm,
			NavSys,
			ProjectExtentCm);
	}
	else
	{
		OutputPoints.Reserve(CornerPoints.Num());
		for (const FNavPathPoint& Point : CornerPoints)
		{
			OutputPoints.Add(Point.Location);
		}
	}

	if (OutputPoints.Num() == 0)
	{
		return TEXT("{\"ok\":false,\"error\":\"empty_path\"}");
	}

	if (bValidateClearance && PlanningObstacles.Num() > 0 && MinCenterClearanceCm > 0.0f)
	{
		float MinClearanceCm = 0.0f;
		FString WorstObstacleId;
		int32 WorstPointIndex = INDEX_NONE;
		const bool bClearanceOk = ValidatePathClearance(
			OutputPoints,
			PlanningObstacles,
			MinCenterClearanceCm,
			ClearanceSegmentSampleCm,
			NavSys,
			ProjectExtentCm,
			MinClearanceCm,
			WorstObstacleId,
			WorstPointIndex);
		if (!bClearanceOk)
		{
			return FString::Printf(
				TEXT("{\"ok\":false,\"error\":\"clearance_violation\","
					 "\"min_center_clearance_cm\":%.3f,"
					 "\"required_center_clearance_cm\":%.3f,"
					 "\"worst_obstacle_id\":\"%s\","
					 "\"worst_point_index\":%d,"
					 "\"corner_point_count\":%d,"
					 "\"output_point_count\":%d}"),
				MinClearanceCm,
				MinCenterClearanceCm,
				*WorstObstacleId,
				WorstPointIndex,
				CornerPoints.Num(),
				OutputPoints.Num());
		}
	}

	const FString PointsJson = PointsToJson(OutputPoints);
	return FString::Printf(
		TEXT("{\"ok\":true,\"agent_radius_cm\":%.3f,"
			 "\"min_center_clearance_cm\":%.3f,"
			 "\"resample_spacing_cm\":%.3f,"
			 "\"corner_point_count\":%d,"
			 "\"point_count\":%d,"
			 "\"points\":%s}"),
		AgentProps.AgentRadius,
		MinCenterClearanceCm,
		bResampleOnNavPath ? ResampleSpacingCm : 0.0f,
		CornerPoints.Num(),
		OutputPoints.Num(),
		*PointsJson);
}

FString ANavQueryService::NavFindPath(
	float StartX,
	float StartY,
	float StartZ,
	float EndX,
	float EndY,
	float EndZ)
{
	return NavFindPathInternal(
		StartX,
		StartY,
		StartZ,
		EndX,
		EndY,
		EndZ,
		DefaultAgentRadiusCm,
		0.0f,
		0.0f,
		false,
		false);
}

FString ANavQueryService::NavFindPathWithRadius(
	float StartX,
	float StartY,
	float StartZ,
	float EndX,
	float EndY,
	float EndZ,
	float AgentRadiusCm)
{
	return NavFindPathInternal(
		StartX,
		StartY,
		StartZ,
		EndX,
		EndY,
		EndZ,
		AgentRadiusCm,
		0.0f,
		0.0f,
		false,
		false);
}

FString ANavQueryService::NavFindPathValidated(
	float StartX,
	float StartY,
	float StartZ,
	float EndX,
	float EndY,
	float EndZ,
	float AgentRadiusCm,
	float MinCenterClearanceCm,
	float ResampleSpacingCm)
{
	return NavFindPathInternal(
		StartX,
		StartY,
		StartZ,
		EndX,
		EndY,
		EndZ,
		AgentRadiusCm,
		MinCenterClearanceCm,
		ResampleSpacingCm,
		true,
		ResampleSpacingCm > 0.0f);
}

FString ANavQueryService::NavRegisterPlanningObstacle(
	const FString& ObstacleId,
	float CenterX,
	float CenterY,
	float HalfExtentX,
	float HalfExtentY)
{
	if (ObstacleId.IsEmpty())
	{
		return TEXT("{\"ok\":false,\"error\":\"empty_id\"}");
	}

	const float SafeHalfX = FMath::Max(1.0f, HalfExtentX);
	const float SafeHalfY = FMath::Max(1.0f, HalfExtentY);

	for (FNavPlanningObstacle& Obstacle : PlanningObstacles)
	{
		if (Obstacle.Id == ObstacleId)
		{
			Obstacle.Cx = CenterX;
			Obstacle.Cy = CenterY;
			Obstacle.HalfX = SafeHalfX;
			Obstacle.HalfY = SafeHalfY;
			return FString::Printf(
				TEXT("{\"ok\":true,\"id\":\"%s\",\"cx\":%.3f,\"cy\":%.3f,"
					 "\"half_x\":%.3f,\"half_y\":%.3f,\"updated\":true}"),
				*ObstacleId,
				CenterX,
				CenterY,
				SafeHalfX,
				SafeHalfY);
		}
	}

	FNavPlanningObstacle Obstacle;
	Obstacle.Id = ObstacleId;
	Obstacle.Cx = CenterX;
	Obstacle.Cy = CenterY;
	Obstacle.HalfX = SafeHalfX;
	Obstacle.HalfY = SafeHalfY;
	PlanningObstacles.Add(Obstacle);

	return FString::Printf(
		TEXT("{\"ok\":true,\"id\":\"%s\",\"cx\":%.3f,\"cy\":%.3f,"
			 "\"half_x\":%.3f,\"half_y\":%.3f,\"count\":%d}"),
		*ObstacleId,
		CenterX,
		CenterY,
		SafeHalfX,
		SafeHalfY,
		PlanningObstacles.Num());
}

FString ANavQueryService::NavIsReachable(
	float StartX,
	float StartY,
	float StartZ,
	float EndX,
	float EndY,
	float EndZ)
{
	const FString PathJson = NavFindPath(StartX, StartY, StartZ, EndX, EndY, EndZ);
	const bool bReachable = PathJson.Contains(TEXT("\"ok\":true"));
	return FString::Printf(
		TEXT("{\"reachable\":%s}"),
		bReachable ? TEXT("true") : TEXT("false"));
}

FString ANavQueryService::GetActorBoundsJson(const FString& ActorName)
{
	AActor* Actor = FindActorByNameOrLabel(ActorName);
	if (!IsValid(Actor))
	{
		return FString::Printf(
			TEXT("{\"ok\":false,\"error\":\"actor_not_found\",\"actor\":\"%s\"}"),
			*ActorName);
	}

	FVector Origin = FVector::ZeroVector;
	FVector Extent = FVector::ZeroVector;
	Actor->GetActorBounds(false, Origin, Extent, false);

	return FString::Printf(
		TEXT("{\"ok\":true,\"actor\":\"%s\",\"cx\":%.3f,\"cy\":%.3f,\"cz\":%.3f,"
			 "\"half_x\":%.3f,\"half_y\":%.3f,\"half_z\":%.3f}"),
		*ActorName,
		Origin.X,
		Origin.Y,
		Origin.Z,
		Extent.X,
		Extent.Y,
		Extent.Z);
}

static UNavModifierComponent* GetBoxObstacleModifier(AActor* Actor)
{
	return Actor ? Actor->FindComponentByClass<UNavModifierComponent>() : nullptr;
}

AActor* ANavQueryService::GetOrCreateBoxObstacle(const FString& ObstacleId)
{
	if (ObstacleId.IsEmpty())
	{
		return nullptr;
	}

	if (TObjectPtr<AActor>* Existing = BoxObstacles.Find(ObstacleId))
	{
		if (IsValid(*Existing))
		{
			return *Existing;
		}
		BoxObstacles.Remove(ObstacleId);
	}

	UWorld* World = GetWorld();
	if (!World)
	{
		return nullptr;
	}

	FActorSpawnParameters Params;
	Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	AActor* BoxActor = World->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity, Params);
	if (!IsValid(BoxActor))
	{
		return nullptr;
	}

	USceneComponent* Root = NewObject<USceneComponent>(
		BoxActor,
		USceneComponent::StaticClass(),
		TEXT("Root"),
		RF_Transactional);
	Root->SetMobility(EComponentMobility::Movable);
	BoxActor->SetRootComponent(Root);
	Root->RegisterComponent();

	UNavModifierComponent* Modifier = NewObject<UNavModifierComponent>(
		BoxActor,
		UNavModifierComponent::StaticClass(),
		TEXT("NavModifier"),
		RF_Transactional);
	Modifier->SetAreaClass(UNavArea_Obstacle::StaticClass());
	Modifier->RegisterComponent();

	BoxActor->SetActorHiddenInGame(true);
	BoxObstacles.Add(ObstacleId, BoxActor);
	return BoxActor;
}

static void SyncBoxObstacleTransform(
	AActor* BoxActor,
	const FVector& Center,
	const FVector& HalfExtents,
	UNavigationSystemV1* NavSys)
{
	if (!IsValid(BoxActor))
	{
		return;
	}

	BoxActor->SetActorLocation(Center, false, nullptr, ETeleportType::TeleportPhysics);
	if (UNavModifierComponent* Modifier = GetBoxObstacleModifier(BoxActor))
	{
		Modifier->FailsafeExtent = HalfExtents;
		Modifier->SetAreaClass(UNavArea_Obstacle::StaticClass());
		Modifier->RefreshNavigationModifiers();
	}

	const FBox Bounds = FBox::BuildAABB(Center, HalfExtents);
	if (NavSys && Bounds.IsValid)
	{
		NavSys->AddDirtyArea(Bounds, ENavigationDirtyFlag::All);
	}
}

FString ANavQueryService::NavRegisterBoxObstacle(
	const FString& ObstacleId,
	float CenterX,
	float CenterY,
	float CenterZ,
	float HalfExtentX,
	float HalfExtentY,
	float HalfExtentZ)
{
	AActor* BoxActor = GetOrCreateBoxObstacle(ObstacleId);
	if (!IsValid(BoxActor))
	{
		return TEXT("{\"ok\":false,\"error\":\"spawn_modifier_failed\"}");
	}

	const FVector Center(CenterX, CenterY, CenterZ);
	const float SafeHalfX = FMath::Max(5.0f, HalfExtentX);
	const float SafeHalfY = FMath::Max(5.0f, HalfExtentY);
	const float SafeHalfZ = FMath::Max(5.0f, HalfExtentZ);
	const FVector HalfExtents(SafeHalfX, SafeHalfY, SafeHalfZ);

	UNavigationSystemV1* NavSys = GetNavSys(GetWorld());
	SyncBoxObstacleTransform(BoxActor, Center, HalfExtents, NavSys);

	FVector BoundsOrigin = Center;
	FVector BoundsExtent = HalfExtents;
	BoxActor->GetActorBounds(false, BoundsOrigin, BoundsExtent, false);
	if (BoundsExtent.IsNearlyZero())
	{
		BoundsOrigin = Center;
		BoundsExtent = HalfExtents;
	}

	return FString::Printf(
		TEXT("{\"ok\":true,\"id\":\"%s\",\"cx\":%.3f,\"cy\":%.3f,\"cz\":%.3f,"
			 "\"half_x\":%.3f,\"half_y\":%.3f,\"half_z\":%.3f,"
			 "\"actual_cx\":%.3f,\"actual_cy\":%.3f,\"actual_cz\":%.3f,"
			 "\"actual_half_x\":%.3f,\"actual_half_y\":%.3f,\"actual_half_z\":%.3f}"),
		*ObstacleId,
		CenterX,
		CenterY,
		CenterZ,
		SafeHalfX,
		SafeHalfY,
		SafeHalfZ,
		BoundsOrigin.X,
		BoundsOrigin.Y,
		BoundsOrigin.Z,
		BoundsExtent.X,
		BoundsExtent.Y,
		BoundsExtent.Z);
}

FString ANavQueryService::NavClearBoxObstacles()
{
	int32 Removed = 0;
	for (TPair<FString, TObjectPtr<AActor>>& Pair : BoxObstacles)
	{
		if (IsValid(Pair.Value))
		{
			Pair.Value->Destroy();
			Removed++;
		}
	}
	BoxObstacles.Empty();
	PlanningObstacles.Empty();
	return FString::Printf(TEXT("{\"ok\":true,\"removed\":%d}"), Removed);
}

FString ANavQueryService::NavRebuild()
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return TEXT("{\"ok\":false,\"error\":\"no_world\"}");
	}

	UNavigationSystemV1* NavSys = GetNavSys(World);
	if (!NavSys)
	{
		return TEXT("{\"ok\":false,\"error\":\"no_navsys\"}");
	}

	const float Margin = FMath::Max(0.0f, RebuildDirtyMarginCm);
	FBox DirtyBounds(ForceInit);
	bool bHasDirtyBounds = false;
	for (TPair<FString, TObjectPtr<AActor>>& Pair : BoxObstacles)
	{
		if (!IsValid(Pair.Value))
		{
			continue;
		}
		UNavModifierComponent* Modifier = GetBoxObstacleModifier(Pair.Value);
		if (Modifier)
		{
			Modifier->SetAreaClass(UNavArea_Obstacle::StaticClass());
			Modifier->RefreshNavigationModifiers();
		}
		FBox Bounds = Pair.Value->GetComponentsBoundingBox(true);
		if (!Bounds.IsValid && Modifier)
		{
			Bounds = FBox::BuildAABB(Pair.Value->GetActorLocation(), Modifier->FailsafeExtent);
		}
		if (!Bounds.IsValid)
		{
			continue;
		}
		Bounds = Bounds.ExpandBy(Margin);
		DirtyBounds += Bounds;
		bHasDirtyBounds = true;
	}
	if (bHasDirtyBounds)
	{
		NavSys->AddDirtyArea(DirtyBounds, ENavigationDirtyFlag::All);
	}
	if (const ANavigationData* NavData = NavSys->GetDefaultNavDataInstance(
		FNavigationSystem::DontCreate))
	{
		const FBox NavBounds = NavData->GetBounds();
		if (NavBounds.IsValid)
		{
			NavSys->AddDirtyArea(NavBounds.ExpandBy(Margin), ENavigationDirtyFlag::All);
		}
	}

	NavSys->Build();
	return FString::Printf(
		TEXT("{\"ok\":true,\"dirty_margin_cm\":%.3f,\"planning_obstacles\":%d}"),
		Margin,
		PlanningObstacles.Num());
}

FString ANavQueryService::NavRebuildDirtyRegion(
	float MinX,
	float MinY,
	float MinZ,
	float MaxX,
	float MaxY,
	float MaxZ,
	float MarginCm)
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return TEXT("{\"ok\":false,\"error\":\"no_world\"}");
	}

	UNavigationSystemV1* NavSys = GetNavSys(World);
	if (!NavSys)
	{
		return TEXT("{\"ok\":false,\"error\":\"no_navsys\"}");
	}

	FBox DirtyBox(ForceInit);
	DirtyBox += FVector(MinX, MinY, MinZ);
	DirtyBox += FVector(MaxX, MaxY, MaxZ);
	if (!DirtyBox.IsValid)
	{
		return TEXT("{\"ok\":false,\"error\":\"invalid_dirty_region\"}");
	}

	const float Margin = FMath::Max(0.0f, MarginCm);
	NavSys->AddDirtyArea(DirtyBox.ExpandBy(Margin), ENavigationDirtyFlag::All);
	NavSys->Build();
	const FVector Size = DirtyBox.GetSize();
	return FString::Printf(
		TEXT("{\"ok\":true,\"local\":true,\"dirty_margin_cm\":%.3f,"
			 "\"dirty_size_x_cm\":%.3f,\"dirty_size_y_cm\":%.3f,\"dirty_size_z_cm\":%.3f}"),
		Margin,
		Size.X,
		Size.Y,
		Size.Z);
}
