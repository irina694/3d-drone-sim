//reference: https://continuebreak.com/articles/blueprints-helicopter-to-cpp-ue5/

#include "DronePawn.h"

#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Math/UnrealMathUtility.h"

#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Widgets/SOverlay.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Blueprint/UserWidget.h"

#include "Engine/Canvas.h"
#include "Engine/TextureRenderTarget2D.h"

#include "imgui.h"
#include "Kismet/GameplayStatics.h"

const FVector start = FVector(0, 0, 1000);
TArray<FVector> make_test_dests()
{
	const int step = 1000;
	const int z_step = 200;
	TArray<FVector> dests;
	dests.Add(start);
	for (int i = 0; i < 1000; i++)
	{
		bool z = FMath::RandBool();
		bool x = FMath::RandBool();
		bool y = FMath::RandBool();
		// get the last point
		FVector last = dests[dests.Num() - 1];
		float z_base = 1000;
		// add a new point
		dests.Add(FVector(last.X + (x ? step : -step), last.Y + (y ? step : -step), z ? z_base + z_step : z_base - z_step));
	}
	return dests;
}

ADronePawn::ADronePawn()
{
	const float droneSize = 105;
	const float thrusterHeight = 22;
	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	//Add these 2 lines below
	DroneBodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Static Mesh"));
	SetRootComponent(DroneBodyMesh);

	DroneCamMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Cam Mesh"));
	DroneCamMesh->AttachToComponent(DroneBodyMesh, FAttachmentTransformRules::KeepWorldTransform);

	SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("Spring Arm"));
	SpringArm->AttachToComponent(DroneBodyMesh, FAttachmentTransformRules::KeepWorldTransform);

	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->AttachToComponent(SpringArm, FAttachmentTransformRules::KeepWorldTransform);

	CameraFPV = CreateDefaultSubobject<UCameraComponent>(TEXT("CameraFPV"));
	CameraFPV->AttachToComponent(DroneBodyMesh, FAttachmentTransformRules::KeepWorldTransform);

	CameraFPV->SetRelativeLocation(FVector(0, 0, -80));

	
	Thruster_Front_Left_Up = CreateDefaultSubobject<UPhysicsThrusterComponent>(TEXT("Thruster FLU"));
	Thruster_Front_Left_Up->AttachToComponent(DroneBodyMesh, FAttachmentTransformRules::KeepWorldTransform);

	Thruster_Front_Right_Up = CreateDefaultSubobject<UPhysicsThrusterComponent>(TEXT("Thruster FRU"));
	Thruster_Front_Right_Up->AttachToComponent(DroneBodyMesh, FAttachmentTransformRules::KeepWorldTransform);

	Thruster_Back_Left_Up = CreateDefaultSubobject<UPhysicsThrusterComponent>(TEXT("Thruster BLU"));
	Thruster_Back_Left_Up->AttachToComponent(DroneBodyMesh, FAttachmentTransformRules::KeepWorldTransform);

	Thruster_Back_Right_Up = CreateDefaultSubobject<UPhysicsThrusterComponent>(TEXT("Thruster BRU"));
	Thruster_Back_Right_Up->AttachToComponent(DroneBodyMesh, FAttachmentTransformRules::KeepWorldTransform);

	Thruster_Front_Left_Up_Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Thruster FLU Mesh"));
	Thruster_Front_Left_Up_Mesh->AttachToComponent(DroneBodyMesh, FAttachmentTransformRules::KeepWorldTransform);

	Thruster_Front_Right_Up_Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Thruster FRU Mesh"));
	Thruster_Front_Right_Up_Mesh->AttachToComponent(DroneBodyMesh, FAttachmentTransformRules::KeepWorldTransform);

	Thruster_Back_Left_Up_Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Thruster BLU Mesh"));
	Thruster_Back_Left_Up_Mesh->AttachToComponent(DroneBodyMesh, FAttachmentTransformRules::KeepWorldTransform);

	Thruster_Back_Right_Up_Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Thruster BRU Mesh"));
	Thruster_Back_Right_Up_Mesh->AttachToComponent(DroneBodyMesh, FAttachmentTransformRules::KeepWorldTransform);


	//Not setting mesh asset in here, 
	//because don't want to deal with having mesh asset always in a predetermined location
	DroneBodyMesh->SetSimulatePhysics(true);
	DroneBodyMesh->SetLinearDamping(1);
	DroneBodyMesh->SetAngularDamping(1);
	
	
	Thruster_Front_Right_Up_Mesh->MoveComponent(FVector(droneSize, droneSize, thrusterHeight), FRotator(0, 0, 0), false);
	Thruster_Front_Left_Up_Mesh->MoveComponent(FVector(droneSize, -droneSize, thrusterHeight), FRotator(0, 0, 0), false);
	
	Thruster_Back_Right_Up_Mesh->MoveComponent(FVector(-droneSize, droneSize, thrusterHeight), FRotator(0, 0, 0), false);
	Thruster_Back_Left_Up_Mesh->MoveComponent(FVector(-droneSize, -droneSize, thrusterHeight), FRotator(0, 0, 0), false);

	Rotors[0] = { Thruster_Front_Left_Up, Thruster_Front_Left_Up_Mesh };
	Rotors[1] = { Thruster_Front_Right_Up,Thruster_Front_Right_Up_Mesh };
	Rotors[2] = { Thruster_Back_Left_Up,Thruster_Back_Left_Up_Mesh };
	Rotors[3] = { Thruster_Back_Right_Up,Thruster_Back_Right_Up_Mesh };
	
	for (Rotor& r : Rotors) {
		r.Thruster->AttachToComponent(r.Mesh, FAttachmentTransformRules::KeepWorldTransform); // attach thrusters to rotor
		r.Thruster->SetRelativeRotation(FRotator(-90, 0, 0)); // rotate thrusters to their own mesh
		r.Thruster->bAutoActivate = true;
		//r.Mesh->SetSimulatePhysics(true);
	}


	SpringArm->TargetArmLength = 800;
	SpringArm->SetRelativeRotation(FRotator(-20, 0, 0));
	SpringArm->bDoCollisionTest = false;
	SpringArm->bInheritPitch = false;
	SpringArm->bInheritRoll = false;


	
	//Put in here just for testing to automatically possess pawn
	this->AutoPossessPlayer = EAutoReceiveInput::Player0;

	this->controller = new DroneController(this);

	Input_ToggleImguiInput = CreateDefaultSubobject<UInputComponent>(TEXT("Toggle Imgui Input"));
	Input_ToggleImguiInput->BindKey(EKeys::I, IE_Pressed, this, &ADronePawn::ToggleImguiInput).bExecuteWhenPaused = true;

	Input_ToggleHoverMode = CreateDefaultSubobject<UInputComponent>(TEXT("Toggle Hover Mode"));
	Input_ToggleHoverMode->BindKey(EKeys::H, IE_Pressed, this, &ADronePawn::ToggleHoverMode).bExecuteWhenPaused = true;

	Input_ToggleFPV = CreateDefaultSubobject<UInputComponent>(TEXT("Toggle FPV"));
	Input_ToggleFPV->BindKey(EKeys::C, IE_Pressed, this, &ADronePawn::SwitchCamera).bExecuteWhenPaused = true;

}

// Called when the game starts or when spawned
void ADronePawn::BeginPlay()
{
	Super::BeginPlay();

	for (Rotor& r : Rotors) {
		r.Thruster->ThrustStrength = 0;
	}
	// for testing
	this->controller->AddNavPlan("TestPlan", make_test_dests());
	this->controller->SetNavPlan("TestPlan");

	for (int i = 0; i < 4; i++) {
		Rotors[i].Thruster->ThrustStrength = 0;
		Rotors[i].Mesh->GetBodyInstance()->SetMassOverride(0.f);
		Rotors[i].Mesh->GetBodyInstance()->UpdateMassProperties();
	}
	
	DroneBodyMesh->GetBodyInstance()->SetMassOverride(.7f);
	DroneBodyMesh->GetBodyInstance()->SetMassScale();
	
	DroneBodyMesh->GetBodyInstance()->UpdateMassProperties();

	DroneCamMesh->GetBodyInstance()->SetMassOverride(0);
	DroneCamMesh->GetBodyInstance()->UpdateMassProperties();
	
	controller->Reset();
	CameraFPV->SetActive(false);
	Camera->SetActive(true);

}

// Called every frame
void ADronePawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	UpdateAnimation(DeltaTime);
	UpdateControl(DeltaTime);
	APlayerController* PlayerController = Cast<APlayerController>(GetController());

}

// Called to bind functionality to input
void ADronePawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	PlayerInputComponent->BindAxis("Mouse Right", this, &ADronePawn::MouseRight);
	PlayerInputComponent->BindAxis("Mouse Up", this, &ADronePawn::MouseUp);
}
#define EPSILON 0.0001f
FRotator FPVCameraRotation = FRotator::ZeroRotator;
void ADronePawn::MouseRight(float Value)
{
	if (Camera->IsActive()) {
		SpringArm->AddRelativeRotation(FRotator(0, Value, 0));
	}
	else {
		FPVCameraRotation.Yaw += Value;
		CameraFPV->SetRelativeRotation(FRotator(FPVCameraRotation.Pitch, FPVCameraRotation.Yaw, 0.0f));
	}
}

void ADronePawn::MouseUp(float Value)
{
	if (Camera->IsActive()) {
		SpringArm->AddRelativeRotation(FRotator(Value, 0, 0));
	}
	else {
		FPVCameraRotation.Pitch += Value;
		FPVCameraRotation.Pitch = FMath::Clamp(FPVCameraRotation.Pitch, -90.0f, 90.0f);
		CameraFPV->SetRelativeRotation(FRotator(FPVCameraRotation.Pitch, FPVCameraRotation.Yaw, 0.0f));
	}
}

inline void ADronePawn::UpdateAnimation(float DeltaTime)
{
	// Rotor animation
	for (Rotor& r : Rotors) {
		r.animate(DeltaTime);
	}
}

inline void ADronePawn::UpdateControl(float DeltaTime)
{
	this->controller->Update(DeltaTime);
}


ADronePawn::DroneController::DroneController(ADronePawn* a_pawn)
{
	this->pawn = a_pawn;
	x_pid = new PIDController();
	x_pid->SetLimits(-max_pid_out, max_pid_out);

	y_pid = new PIDController();
	y_pid->SetLimits(-max_pid_out, max_pid_out);

	z_pid = new PIDController();
	z_pid->SetLimits(-max_pid_out, max_pid_out);
	
	pitch_pid = new PIDController();
	roll_pid = new PIDController();
	
	roll_pid->SetLimits(-max_pid_out, max_pid_out);
	pitch_pid->SetLimits(-max_pid_out, max_pid_out);
	roll_pid->rotational = true;
	pitch_pid->rotational = true;
}

inline float CalculateThrustFrontLeft(float x_output, float y_output, float z_output, float roll_output, float pitch_output)
{
	return z_output - x_output + y_output + roll_output + pitch_output;
}
inline float CalculateThrustFrontRight(float x_output, float y_output, float z_output, float roll_output, float pitch_output)
{
	return z_output - x_output - y_output - roll_output + pitch_output;
}
inline float CalculateThrustRearLeft(float x_output, float y_output, float z_output, float roll_output, float pitch_output)
{
	return z_output + x_output + y_output + roll_output - pitch_output;
}
inline float CalculateThrustRearRight(float x_output, float y_output, float z_output, float roll_output, float pitch_output)
{
	return z_output + x_output - y_output - roll_output - pitch_output;
}

inline FVector CalculateDesiredVelocity(const FVector& error, float max_velocity) {
	FVector desired_velocity = error.GetSafeNormal() * max_velocity;
	return desired_velocity;
}

inline float CalculateDesiredRoll(const FVector& normalizedError, const FVector& droneForwardVector, float max_tilt, float altitudeThreshold) {
	FVector horizontalError = FVector(normalizedError.X, normalizedError.Y, 0.0f);
	FVector horizontalNormalizedError = horizontalError.GetSafeNormal();

	if (FMath::Abs(normalizedError.Z) > altitudeThreshold) {
		return 0.0f;
	}
	else {
		float calculatedRoll = FMath::Atan2(normalizedError.Y, FVector::DotProduct(horizontalNormalizedError, droneForwardVector)) * FMath::RadiansToDegrees(1);
		return FMath::Clamp(calculatedRoll, -max_tilt, max_tilt);
	}
}


inline float CalculateDesiredPitch(const FVector& normalizedError, const FVector& droneForwardVector, float max_tilt, float altitudeThreshold) {
	FVector horizontalError = FVector(normalizedError.X, normalizedError.Y, 0.0f);
	FVector horizontalNormalizedError = horizontalError.GetSafeNormal();

	if (FMath::Abs(normalizedError.Z) > altitudeThreshold) {
		return 0.0f;
	}
	else {
		float calculatedPitch = FMath::Atan2(-normalizedError.X, FVector::DotProduct(horizontalNormalizedError, droneForwardVector)) * FMath::RadiansToDegrees(1);
		return FMath::Clamp(calculatedPitch, -max_tilt, max_tilt);
	}
}

/* Functional programming >> OOP */
float max_velocity = 200.0f;  // the maximum desired velocity of each axis, The higher this value the faster and less stable the drone becomes
float max_tilt = 30.f; // the maximum tilt angle of the drone.


float altitude_threshold = .6f; // if the relative z is more than this we set roll and pitch to 0, wait for the drone to ascend.
float deadband_size = 150.0f; // in hovermode, if the error is less than this, we set the error to 0
float min_altitude = 800.f; // if the altitude is less than this in the beginning, the drone must ascend to this altitude.

void ADronePawn::DroneController::Update(double a_deltaTime)
{
	if (this->curNavPlan == nullptr || curPos >= curNavPlan->waypoints.Num()) { // at dest or don't have a nav plan
		return;
	}

	float mass = pawn->DroneBodyMesh->GetMass();
	auto currLoc = pawn->GetActorLocation();

	// Get the current waypoint from the nav plan
	FVector waypoint = curNavPlan->waypoints[curPos];

	if (!altitudeReached) { // drone must ascend
		if (currLoc.Z < min_altitude) {
			waypoint = FVector(currLoc.X, currLoc.Y, min_altitude); // Set waypoint to be directly above the drone
		}
		else {
			altitudeReached = true;
		}
	}
	
	// show the waypoint on the scene
	static bool Debug_DrawDroneCollisionSphere = true;
	static bool Debug_DrawDroneWaypoint = true;
	if (Debug_DrawDroneCollisionSphere) {
		DrawDebugSphere(pawn->GetWorld(), pawn->GetActorLocation(), pawn->DroneBodyMesh->GetCollisionShape().GetSphereRadius(), 10, FColor::Red, false, 0.1f);
	}
	if (Debug_DrawDroneWaypoint) {
		DrawDebugSphere(pawn->GetWorld(), waypoint, ACCEPTABLE_DIST, 10, FColor::Red, false, 0.1f);
		DrawDebugLine(pawn->GetWorld(), pawn->GetActorLocation(), waypoint, FColor::Green, false, 0.1f);
	}
		// Calculate the error between the current location and the waypoint
	FVector error = waypoint - currLoc;

	// Check if the drone has reached the waypoint
	if (error.Size() < pawn->DroneBodyMesh->GetCollisionShape().GetSphereRadius()) {
		// Move to the next waypoint
		if (!altitudeReached) { // fix 
			altitudeReached = true;
		}
		else {
			curPos++;
		}
		if (curPos >= curNavPlan->waypoints.Num()) {
			// Reached the end of the nav plan
			curNavPlan = nullptr;
			curPos = 0;
		}
	}
	FVector normalizedError = error.GetSafeNormal();
	FVector droneForwardVector = pawn->GetActorForwardVector();

	// Calculate the desired roll, pitch, and yaw angles based on the error vector
	float desired_roll, desired_pitch;

	desired_roll = CalculateDesiredRoll(normalizedError, droneForwardVector, hoverMode ? 0 : max_tilt, altitude_threshold);
	desired_pitch = CalculateDesiredPitch(normalizedError, droneForwardVector, hoverMode ? 0 : max_tilt, altitude_threshold);
	
 // Calculate the errors in roll, pitch, and yaw angles
	FRotator current_rotation = pawn->GetActorRotation();

	float roll_error = desired_roll - current_rotation.Roll;
	float pitch_error = desired_pitch - current_rotation.Pitch;


	// Update the roll, pitch PID controllers
	float roll_output = roll_pid->Calculate(roll_error, a_deltaTime);
	float pitch_output = pitch_pid->Calculate(pitch_error, a_deltaTime);
	

	// Update the PID controllers for each axis
	// Calculate the desired velocities for each axis
	FVector desired_velocity = CalculateDesiredVelocity(error, hoverMode ? 0 : max_velocity);

	FVector current_velocity = pawn->GetVelocity();

	float x_output = x_pid->Calculate(desired_velocity.X - current_velocity.X, a_deltaTime);
	float y_output = y_pid->Calculate(desired_velocity.Y - current_velocity.Y, a_deltaTime);
	float z_output = z_pid->Calculate(desired_velocity.Z - current_velocity.Z, a_deltaTime);

	// putting everything together
	float thrust_front_left = CalculateThrustFrontLeft(x_output, y_output, z_output, roll_output, pitch_output);
	float thrust_front_right = CalculateThrustFrontRight(x_output, y_output, z_output, roll_output, pitch_output);
	float thrust_rear_left = CalculateThrustRearLeft(x_output, y_output, z_output, roll_output, pitch_output);
	float thrust_rear_right = CalculateThrustRearRight(x_output, y_output, z_output, roll_output, pitch_output);

	const float mult = 1;
	// Update the thrust of each rotor
	pawn->Rotors[0].Thruster->ThrustStrength = mass * mult * thrust_front_left;
	pawn->Rotors[1].Thruster->ThrustStrength = mass * mult * thrust_front_right;
	pawn->Rotors[2].Thruster->ThrustStrength = mass * mult * thrust_rear_left;
	pawn->Rotors[3].Thruster->ThrustStrength = mass * mult * thrust_rear_right;

// everything below is UI
	ImGui::Begin("Drone Controller");

	ImGui::SliderFloat("Max velocity", &max_velocity, 0.0f, 600.0f);
	ImGui::SliderFloat("Max tilt angle", &max_tilt, 0.0f, 45.0f);

	ImGui::Separator();
	ImGui::Text("Debug Draw");
	ImGui::Checkbox("Drone Collision Sphere", &Debug_DrawDroneCollisionSphere);
	ImGui::Checkbox("Drone Waypoint", &Debug_DrawDroneWaypoint);
	ImGui::Separator();

	ImGui::Text("Thruster Power");
	
	ImGui::SliderFloat("Front Left", &thrust_front_left, -max_pid_out, max_pid_out);
	ImGui::SliderFloat("Front Right", &thrust_front_right, -max_pid_out, max_pid_out);
	ImGui::SliderFloat("Rear Left", &thrust_rear_left, -max_pid_out, max_pid_out);
	ImGui::SliderFloat("Rear Right", &thrust_rear_right, -max_pid_out, max_pid_out);
	ImGui::Separator();

	ImGui::Text("Desired Roll: %f", desired_roll);
	ImGui::SameLine();
	ImGui::Text("Current Roll: %f", current_rotation.Roll);

	ImGui::Text("Desired Pitch: %f", desired_pitch);
	ImGui::SameLine();
	ImGui::Text("Current pitch: %f", current_rotation.Pitch);

	ImGui::Text("Error Roll: %f", roll_error);
	ImGui::SameLine();
	ImGui::Text("roll_output: %f", roll_output);

	ImGui::Text("pitch_error: %f", pitch_error);
	ImGui::SameLine();
	ImGui::Text("pitch_output: %f", pitch_output);
	
	ImGui::Separator();
	ImGui::Text("Desired X, Y, Z: %f, %f, %f", waypoint.X, waypoint.Y, waypoint.Z);
	ImGui::Text("Current X, Y, Z: %f, %f, %f", currLoc.X, currLoc.Y, currLoc.Z);
	ImGui::Text("Error   X, Y, Z: %f, %f, %f", error.X, error.Y, error.Z);
	ImGui::Spacing();
	ImGui::Text("Desired velocity: %f, %f, %f", desired_velocity.X, desired_velocity.Y, desired_velocity.Z);
	ImGui::Text("Current velocity: %f, %f, %f", current_velocity.X, current_velocity.Y, current_velocity.Z);
	ImGui::Spacing();
	ImGui::Text("Output  X, Y, Z: %f, %f, %f", x_output, y_output, z_output);
	

	ImGui::Separator();
	ImGui::Spacing();
	ImGui::Text("Camera Mode: %s", pawn->CameraFPV->IsActive() ? "First Person" : "Third Person");
	ImGui::Text("Flying Mode: %s", hoverMode ? "Hover" : "Target");
	ImGui::Text("Press \"C\" to switch camera mode");
	ImGui::Text("Press \"H\" to switch flying mode");

	ImGui::Separator();
	if (ImGui::Button("Release Input", ImVec2(200, 100))) {
		pawn->ToggleImguiInput();
	}
	// new window

	ImGui::Begin("PID Settings");
	ImGui::Text("Change the following only if you know what you're doing");
	ImGui::Text("Positional PID");
	ImGui::SliderFloat("P", &PIDController::m_p, 0, 5);
	ImGui::SliderFloat("I", &PIDController::m_i, 0, 1);
	ImGui::SliderFloat("D", &PIDController::m_d, 0, 1);

	ImGui::Text("Rotational PID");
	ImGui::SliderFloat("P2", &PIDController::m_p_2, 0, 5);
	ImGui::SliderFloat("I2", &PIDController::m_i_2, 0, 1);
	ImGui::SliderFloat("D2", &PIDController::m_d_2, 0, 10);
}

void ADronePawn::DroneController::AddNavPlan(FString name, TArray<FVector> waypoints)
{
	NavPlan plan;
	plan.name = name;
	plan.waypoints = waypoints;
	navPlans.Add(plan);
}

void ADronePawn::DroneController::SetNavPlan(FString name)
{
	for (int i = 0; i < navPlans.Num(); i++) {
		if (navPlans[i].name == name) {
			curNavPlan = &navPlans[i];
			curPos = 0;
			return;
		}
	}
}


ADronePawn::DroneController::PIDController::PIDController()
{
	m_min_output = -1.0f;
	m_max_output = 1.0f;
	m_integral = 0.0f;
	m_prev_error = 0.0f;
}

void ADronePawn::DroneController::PIDController::SetGains(float p, float i, float d)
{
	m_p = p;
	m_i = i;
	m_d = d;
}

void ADronePawn::DroneController::PIDController::SetLimits(float min_output, float max_output)
{
	m_min_output = min_output;
	m_max_output = max_output;
}

void ADronePawn::DroneController::Reset()
{
	x_pid->Reset();
	y_pid->Reset();
	z_pid->Reset();
	roll_pid->Reset();
	pitch_pid->Reset();
	curPos = 0;
	altitudeReached = false;
}

void ADronePawn::DroneController::PIDController::Reset()
{
	m_integral = 0.0f;
	m_prev_error = 0.0f;
}

float ADronePawn::DroneController::PIDController::Calculate(float error, float dt)
{
	float p = rotational ? m_p_2: m_p;
	float i = rotational ? m_i_2 : m_i;
	float d = rotational ? m_d_2 : m_d;
	// Calculate the proportional term
	float p_term = p * error;

	// Calculate the integral term
	m_integral += error * dt;
	float i_term = i * m_integral;
	// Calculate the derivative term
	float d_term = d * (error - m_prev_error) / dt;

	// Calculate the output
	float output = p_term + i_term + d_term;

	// Clamp the output to the specified limits
	output = FMath::Clamp(output, m_min_output, m_max_output);

	// Update the previous error
	m_prev_error = error;

	return output;
	
}


void ADronePawn::SwitchCamera()
{
	if (CameraFPV->IsActive())
	{
		// enable 3rd person
		CameraFPV->SetActive(false);
		Camera->SetActive(true);
		DroneCamMesh->SetHiddenInGame(false);
	}
	else
	{
		// enable 1st person
		CameraFPV->SetActive(true);
		Camera->SetActive(false);
		DroneCamMesh->SetHiddenInGame(true);
	}
}

void ADronePawn::ToggleImguiInput()
{
	UGameplayStatics::GetPlayerController(GetWorld(), 0)->ConsoleCommand("ImGui.ToggleInput");
}

void ADronePawn::ToggleHoverMode()
{
	this->controller->hoverMode = !this->controller->hoverMode;
}
