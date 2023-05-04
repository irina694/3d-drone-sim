
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "PhysicsEngine/PhysicsThrusterComponent.h"

#include "Camera/CameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Widgets/SCompoundWidget.h"

#include <Components/Image.h>


#include "Blueprint/UserWidget.h"


#include "DronePawn.generated.h"


UCLASS()
class DRONESIM_API ADronePawn : public APawn
{
	GENERATED_BODY()

private:
#define ACCEPTABLE_DIST 200
	class DroneController
	{
	public:
		class PIDController
		{

		public:
			PIDController();

			void SetGains(float p, float i, float d);
			void SetLimits(float min_output, float max_output);
			void Reset();

			float Calculate(float error, float dt);
			static inline float m_p = 2.5f, m_i = .3f, m_d = .16f; // positional PID
			static inline float m_p_2 = 1.f, m_i_2 = .19f, m_d_2 = 10.f; // rotational PID
			bool rotational = false;

		private:
			float m_min_output;
			float m_max_output;
			float m_integral;
			float m_prev_error;
		};
		struct NavPlan
		{
			TArray<FVector> waypoints; // a series of discrete points to follow
			FString name;
		};
		ADronePawn* pawn;
		DroneController(ADronePawn* a_pawn);
		void Update(double a_deltaTime);
		
		void AddNavPlan(FString name, TArray<FVector> waypoints);
		void SetNavPlan(FString name);
		void Reset();
		bool hoverMode = false;

	private:
		TArray<NavPlan> navPlans;
		NavPlan* curNavPlan;
		int32 curPos; // current position in the nav plan

		PIDController* x_pid;
		PIDController* y_pid;
		PIDController* z_pid;
		PIDController* roll_pid;
		PIDController* pitch_pid;

		bool altitudeReached = false;
		static inline const float max_pid_out = 600.f; // max thrusting power individual rotor can provide
	};
	DroneController* controller;
public:
	// Sets default values for this pawn's properties
	ADronePawn();
	
protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

	UPROPERTY(EditAnywhere)
		UStaticMeshComponent* DroneBodyMesh;

	UPROPERTY(EditAnywhere)
		UStaticMeshComponent* DroneCamMesh;

	UPROPERTY(VisibleAnywhere)
		class USpringArmComponent* SpringArm;

	UPROPERTY(VisibleAnywhere)
		class UCameraComponent* Camera;

	UPROPERTY(VisibleAnywhere)
		class UCameraComponent* CameraFPV;

	UPROPERTY(VisibleAnywhere)
		class UPhysicsThrusterComponent* Thruster_Front_Left_Up;
	UPROPERTY(VisibleAnywhere)
		class UPhysicsThrusterComponent* Thruster_Front_Right_Up;
	UPROPERTY(VisibleAnywhere)
		class UPhysicsThrusterComponent* Thruster_Back_Left_Up;
	UPROPERTY(VisibleAnywhere)
		class UPhysicsThrusterComponent* Thruster_Back_Right_Up;

	float Thruster_Front_Left_Up_AV; // angular velocity

	UPROPERTY(EditAnywhere, Category = "RotorMesh")
		UStaticMeshComponent* Thruster_Front_Left_Up_Mesh;
	UPROPERTY(EditAnywhere, Category = "RotorMesh")
		UStaticMeshComponent* Thruster_Front_Right_Up_Mesh;
	UPROPERTY(EditAnywhere, Category = "RotorMesh")
		UStaticMeshComponent* Thruster_Back_Left_Up_Mesh;
	UPROPERTY(EditAnywhere, Category = "RotorMesh")
		UStaticMeshComponent* Thruster_Back_Right_Up_Mesh;
	UFUNCTION()
		void MouseRight(float Value);

	UFUNCTION()
		void MouseUp(float Value);
	
	UInputComponent* Input_ToggleImguiInput;
	UInputComponent* Input_ToggleHoverMode;
	UInputComponent* Input_ToggleFPV;



	void SwitchCamera();

private:	
	struct Rotor
	{
		UPhysicsThrusterComponent* Thruster;
		float av = 0; // angular velocity
		UStaticMeshComponent* Mesh;
		Rotor(UPhysicsThrusterComponent* Thruster, UStaticMeshComponent* Mesh)
			: Thruster(Thruster), Mesh(Mesh)
		{
		}
		Rotor() { Thruster = nullptr; Mesh = nullptr; }
		inline void animate(float DeltaTime)
		{
			const float mult = 0.1;
			const float inertia = 0.2;
			const float RotorAcceleration = (Thruster->ThrustStrength * mult - av) / inertia;
			av += RotorAcceleration * DeltaTime;
			FRotator Rotation = Mesh->GetRelativeRotation();
			Rotation.Yaw += av * DeltaTime;
			Mesh->SetRelativeRotation(Rotation);
		}
	};

	Rotor Rotors[4]; // FL, FR. BL, BR
private:
	void UpdateAnimation(float DeltaTime);
	void UpdateControl(float DeltaTime);
	
	void ToggleImguiInput();
	void ToggleHoverMode();
};

