// Copy into SimWorld Source (see INSTALL_NATIVE.md).
#include "NavQueryService.h"

#include "Components/SceneComponent.h"
#include "NavigationPath.h"
#include "NavigationSystem.h"
#include "NavFilters/NavigationQueryFilter.h"

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

FString ANavQueryService::NavFindPath(
	float StartX,
	float StartY,
	float StartZ,
	float EndX,
	float EndY,
	float EndZ)
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

	const FPathFindingQuery Query(
		this,
		*NavData,
		StartNav.Location,
		EndNav.Location);
	const FPathFindingResult Result = NavSys->FindPathSync(FNavAgentProperties(), Query);
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

	return FString::Printf(TEXT("{\"ok\":true,\"points\":%s}"), *PointsJson);
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
