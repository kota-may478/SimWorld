// Copy into SimWorld Source (see INSTALL_NATIVE.md).
#include "NavQueryService.h"

#include "Components/BrushComponent.h"
#include "Components/SceneComponent.h"
#include "EngineUtils.h"
#include "NavAreas/NavArea_Obstacle.h"
#include "AI/Navigation/NavigationTypes.h"
#include "NavigationPath.h"
#include "NavigationSystem.h"
#include "NavModifierVolume.h"

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
	float AgentRadiusCm)
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
	const FPathFindingResult Result = NavSys->FindPathSync(AgentProps, Query);
	if (!Result.IsSuccessful() || !Result.Path.IsValid())
	{
		return TEXT("{\"ok\":false,\"error\":\"no_path\"}");
	}

	const TArray<FNavPathPoint>& PathPoints = Result.Path->GetPathPoints();
	FString PointsJson = TEXT("[");
	for (int32 Index = 0; Index < PathPoints.Num(); ++Index)
	{
		const FVector& Loc = PathPoints[Index].Location;
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

	return FString::Printf(
		TEXT("{\"ok\":true,\"agent_radius_cm\":%.3f,\"points\":%s}"),
		AgentProps.AgentRadius,
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
		DefaultAgentRadiusCm);
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
		AgentRadiusCm);
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

ANavModifierVolume* ANavQueryService::GetOrCreateBoxObstacle(const FString& ObstacleId)
{
	if (ObstacleId.IsEmpty())
	{
		return nullptr;
	}

	if (TObjectPtr<ANavModifierVolume>* Existing = BoxObstacles.Find(ObstacleId))
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
	ANavModifierVolume* Volume = World->SpawnActor<ANavModifierVolume>(
		ANavModifierVolume::StaticClass(),
		FTransform::Identity,
		Params);
	if (!IsValid(Volume))
	{
		return nullptr;
	}

	Volume->SetActorHiddenInGame(true);
	Volume->SetActorEnableCollision(false);
	Volume->SetAreaClass(UNavArea_Obstacle::StaticClass());
	if (UBrushComponent* Brush = Volume->GetBrushComponent())
	{
		Brush->SetCanEverAffectNavigation(true);
		Brush->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	}
	BoxObstacles.Add(ObstacleId, Volume);
	return Volume;
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
	ANavModifierVolume* Volume = GetOrCreateBoxObstacle(ObstacleId);
	if (!IsValid(Volume))
	{
		return TEXT("{\"ok\":false,\"error\":\"spawn_modifier_failed\"}");
	}

	const FVector Center(CenterX, CenterY, CenterZ);
	Volume->SetActorLocation(Center, false, nullptr, ETeleportType::TeleportPhysics);

	// Default brush is 100uu half-extent on each axis (200uu cube).
	const float SafeHalfX = FMath::Max(5.0f, HalfExtentX);
	const float SafeHalfY = FMath::Max(5.0f, HalfExtentY);
	const float SafeHalfZ = FMath::Max(5.0f, HalfExtentZ);
	Volume->SetActorScale3D(FVector(
		SafeHalfX / 100.0f,
		SafeHalfY / 100.0f,
		SafeHalfZ / 100.0f));
	Volume->SetAreaClass(UNavArea_Obstacle::StaticClass());
	if (UBrushComponent* Brush = Volume->GetBrushComponent())
	{
		Brush->SetCanEverAffectNavigation(true);
	}

	return FString::Printf(
		TEXT("{\"ok\":true,\"id\":\"%s\",\"cx\":%.3f,\"cy\":%.3f,\"cz\":%.3f,"
			 "\"half_x\":%.3f,\"half_y\":%.3f,\"half_z\":%.3f}"),
		*ObstacleId,
		CenterX,
		CenterY,
		CenterZ,
		SafeHalfX,
		SafeHalfY,
		SafeHalfZ);
}

FString ANavQueryService::NavClearBoxObstacles()
{
	int32 Removed = 0;
	for (TPair<FString, TObjectPtr<ANavModifierVolume>>& Pair : BoxObstacles)
	{
		if (IsValid(Pair.Value))
		{
			Pair.Value->Destroy();
			Removed++;
		}
	}
	BoxObstacles.Empty();
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

	FBox DirtyBounds(ForceInit);
	bool bHasDirtyBounds = false;
	for (TPair<FString, TObjectPtr<ANavModifierVolume>>& Pair : BoxObstacles)
	{
		if (!IsValid(Pair.Value))
		{
			continue;
		}
		Pair.Value->SetAreaClass(UNavArea_Obstacle::StaticClass());
		const FBox Bounds = Pair.Value->GetComponentsBoundingBox(true);
		if (Bounds.IsValid)
		{
			DirtyBounds += Bounds;
			bHasDirtyBounds = true;
		}
	}
	if (bHasDirtyBounds)
	{
		NavSys->AddDirtyArea(DirtyBounds, ENavigationDirtyFlag::All);
	}

	NavSys->Build();
	return TEXT("{\"ok\":true}");
}
