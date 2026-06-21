// Optional native parent for BP_SemanticCollisionProbe (copy into SimWorld Source).
#include "SemanticCollisionProbe.h"

#include "Components/SceneComponent.h"
#include "Engine/World.h"

ASemanticCollisionProbe::ASemanticCollisionProbe()
{
	PrimaryActorTick.bCanEverTick = false;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
}

static bool SweepPointOnChannel(
	UWorld* World,
	const FVector& Center,
	float Radius,
	ECollisionChannel Channel,
	const FCollisionQueryParams& QueryParams,
	FHitResult& OutHit)
{
	const FCollisionShape Sphere = FCollisionShape::MakeSphere(Radius);
	return World->SweepSingleByChannel(
		OutHit,
		Center,
		Center,
		FQuat::Identity,
		Channel,
		Sphere,
		QueryParams);
}

FString ASemanticCollisionProbe::ProbePointHit(float X, float Y, float Z, float RadiusCm)
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return TEXT("{\"hit\":false,\"building\":0,\"object\":0,\"error\":\"no_world\"}");
	}

	const FVector Center(X, Y, Z);
	const float Radius = FMath::Max(0.01f, RadiusCm > 0.f ? RadiusCm : ProbeRadiusCm);
	FCollisionQueryParams QueryParams(SCENE_QUERY_STAT(SemanticCollisionProbe), false, this);

	FHitResult Hit;
	bool bHit = SweepPointOnChannel(World, Center, Radius, ECC_WorldStatic, QueryParams, Hit);
	if (!bHit)
	{
		bHit = SweepPointOnChannel(World, Center, Radius, ECC_WorldDynamic, QueryParams, Hit);
	}

	int32 BuildingHits = 0;
	int32 ObjectHits = 0;
	if (bHit && Hit.GetActor() != nullptr && IsValid(Hit.GetActor()) && Hit.GetActor() != this)
	{
		const FString Name = Hit.GetActor()->GetName();
		if (Name.Contains(TEXT("Building")) || Name.Contains(TEXT("building")))
		{
			BuildingHits = 1;
		}
		else
		{
			ObjectHits = 1;
		}
	}

	const bool bBlocks = bHit && (BuildingHits + ObjectHits) > 0;
	return FString::Printf(
		TEXT("{\"hit\":%s,\"building\":%d,\"object\":%d}"),
		bBlocks ? TEXT("true") : TEXT("false"),
		BuildingHits,
		ObjectHits);
}
