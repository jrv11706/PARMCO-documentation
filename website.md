layout: default

🤖 PARMCO: Phone App RP4 Motor Control

ECSE 4235 Final Project | Group 5: Liam Dabelstein, Jake VanEssendelft

This project is a small, standalone embedded system that allows a user to control a 12V DC motor using a custom-built Android application. The system provides both manual control (Start/Stop, Speed +/-, Direction) and a fully automated, closed-loop PID controller to maintain a specific, user-defined RPM.

The entire project—from the circuit design and C server to the Android app and PID tuning—was created in collaboration with AI assistants (Gemini 2.5 Pro and Claude AI) as required by the course [cite: ECSE4235_FinalProjectF25 (1).pdf].

🏛️ System Architecture

The PARMCO system is built on four main components that communicate in a clear hierarchy:

Android Application: A custom app built in Kotlin/Android Studio [cite: MainActivity.kt]. It provides the user interface (UI), sends text-based commands (e.g., START_STOP, SET_RPM:5000) over Bluetooth, and displays real-time RPM data.

Raspberry Pi 4 (Server): Runs a multithreaded C server (server.c) [cite: server.c] that listens for Bluetooth connections. It parses app commands, controls GPIO pins using the pigpio library, and runs all real-time calculation and control threads.

Motor Control Circuit: A custom-designed H-bridge circuit based on the SN754410 driver, which allows for bidirectional control of the 12V DC motor [cite: https://www.google.com/search?q=image_10d46c.png].

Feedback Sensor: A 3-pin IR sensor detects 3 propeller blades on the motor, providing a pulse to a GPIO pin for precise RPM calculation (3 pulses = 1 revolution) [cite: https://www.google.com/search?q=image_10d46c.png, user].

⚡ Hardware Design: The Motor Circuit

The final circuit [cite: https://www.google.com/search?q=image_10d46c.png] was the result of an iterative design process. The initial AI-generated design from Checkpoint 1 used an optocoupler for isolation, which failed during physical testing in Checkpoint 2 [cite: Checkpoint2.zip/Group 5 - CP2 Timeline of Prompts (Human Written).docx].

The "Tug of War" Hardware Flaw

During hardware validation, we discovered the RP4's 3.3V GPIO pin could not source enough current (only ~2.5mA) to properly drive the 4N25 optocoupler's LED. A 4.7kΩ pull-up resistor on the motor side won the "tug of war," holding the MOSFET gate HIGH and preventing the motor from ever turning off [cite: Checkpoint2.zip/Group 5 - CP2 AI Generated Documentation.docx].

Final Circuit Solution

The optocoupler was removed and replaced with a simple 2N3904 NPN Transistor Buffer.

The RP4's GPIO pin now sends a low-current signal to the transistor's base.

The transistor does the "heavy lifting," sinking the 2.55mA from the pull-up resistor to pull the MOSFET gate LOW, successfully turning the motor off.

This design sacrifices galvanic isolation but results in a simpler, more reliable, and fully functional circuit that meets project requirements [cite: Checkpoint2.zip/Group 5 - CP2 AI Generated Documentation.docx].

Truth Table / Operation

Master Enable (Q1): GPIO LOW = Motor Power ON, GPIO HIGH = Motor Power OFF (Inverted Logic).

Direction (SN754410): 1,2EN=HIGH, 1A=HIGH, 2A=LOW = Forward (A→B).

Speed Control: PWM on 1,2EN pin (GPIO 18) [cite: cp2_keyboard_test.c].

RPM Sensing: IR sensor (GPIO 23) detects 3 pulses per fan revolution [cite: user].

🧠 Software: Raspberry Pi 4 (Server)

The server is a multithreaded C application that manages GPIO, Bluetooth, and real-time control loops.

Headless Bluetooth Automation

A major challenge was making the Bluetooth pairing "headless" (no Pi interaction), as required by the project. This was solved with a 3-part system based on our AI-assisted debugging:

bt-agent.service: A custom systemd service that runs bt-agent -c NoInputNoOutput on boot, forcing the Pi to auto-accept all pairing requests without a PIN [cite: autopair_service.txt].

bluetooth.service (Override): The main Bluetooth service was modified to include the --compat flag, making re-pairing more reliable after a "forget" event [cite: bluetooth_service.txt].

clear-bt-devices.sh: A helper script that clears all paired devices from the Pi's memory [cite: clear-bt-devices.sh]. This script is run by our parmco-server.service on boot and by server.c on every client disconnect [cite: parmco_service.txt, server.c]. This solves the "stale key" bug and ensures a fresh pairing is possible on every connection.

Core Server Logic (server.c)

The C server [cite: server.c] is built on five parallel threads:

Main Thread: Waits for new clients to connect via accept(). When one does, it spawns a client_read_thread and a client_write_thread for that client.

client_read_thread: Listens for incoming commands from the app (e.g., SET_RPM:5000\n). It parses these commands and calls the appropriate motor control functions.

client_write_thread: Runs in a loop, sending the current RPM data back to the app 4 times per second (RPM:5000.0\n).

rpm_calculator_thread: Wakes up 10 times per second to calculate the instantaneous RPM based on pulses from the IR sensor (rpm_callback). It then applies a Complementary Filter (COMPLEMENTARY_ALPHA = 0.85) to smooth the data, providing a stable RPM value while minimizing lag [cite: server.c].

feedback_controller_thread: This is the "brain." It runs the PID controller 10 times per second, comparing filtered_rpm to desired_rpm and adjusting the PWM speed accordingly.

Advanced PID Controller

To achieve stable and accurate RPM, a sophisticated PID controller was implemented and tuned:

PID Control: Uses Proportional, Integral, and Derivative terms [cite: server.c].

Sensor Filtering: Uses the filtered RPM value from the rpm_calculator_thread to prevent reactions to sensor noise.

Gain Scheduling: The controller uses two sets of gains: one for low RPM targets (<4000) and a more aggressive set for high RPM targets (>=4000) to optimize performance across the motor's full range [cite: server.c].

Error Deadband: To prevent oscillation ("hunting") at the target, the controller ignores small errors within a ERROR_DEADBAND of 100 RPM [cite: server.c].

Integral Reset: The integral_error is reset to zero every time the mode is changed or a new RPM is set, preventing "integral wind-up" and ensuring a fast response to new targets [cite: server.c].

📱 Software: Android App (MainActivity.kt)

The Android application is the sole interface for the project. It was written in Kotlin using Android Studio.

Key Features

Bluetooth Scanning: Uses a BroadcastReceiver to scan for Bluetooth devices, finding the Pi by its name (group5pi) [cite: MainActivity.kt].

Connection: Uses a Java reflection hack (createRfcommSocket(1)) to connect directly to the Pi's RFCOMM channel 1. This was necessary because the simple C server does not advertise a formal SDP service [cite: MainActivity.kt].

Two-Way Data:

sendCommand(): Sends string commands to the Pi's outputStream (e.g., START_STOP\n).

startReadThread(): Uses a BufferedReader to listen to the inputStream for RPM:....\n messages, parsing the value and updating the "Actual RPM" text field in real-time.

Robust Connection: The app includes a "Disconnect" button and automatically detects connection loss (via the read() thread failing), triggering a disconnect() function that resets all UI elements to their default state.

Stateful UI: The app's UI is interactive. Toggling "Auto" mode disables the manual speed buttons, and the "START" button toggles its own text to "STOP".

📄 AI-Generated Project Summaries

As required by the project, the following documentation was generated by our AI collaborators (Gemini 2.5 Pro & Claude AI) to summarize the development process at each checkpoint.

<details>
<summary><strong>Checkpoint 1: AI-Generated Documentation (Click to expand)</strong></summary>

From "Group 5 - CP1 AI Generated Documentation.docx" [cite: Group 5 - CP1 AI Generated Documentation.docx]

Project Context: This conversation was part of the ECSE 4235 Final Project (Checkpoint #1) for a university senior-level embedded systems course. The team is designing a motor driver circuit to enable a Raspberry Pi 4 to control a 12V DC motor with bidirectional rotation capability for the PARMCO (Phone APP RP4 Motor Control) system.

Initial Design Review: The conversation began with a review of a preliminary motor driver circuit design...
Critical Design Flaw Identified: Upon reviewing the project requirements, a fundamental issue was identified: the initial design could not reverse motor direction...
Design Evolution:

Version 1: L298N H-Bridge with Multiple Optocouplers

Version 2: Discrete 4-MOSFET H-Bridge

Version 3: Corrected Understanding of H-Bridge ICs
Final Design Solution: The final design uses:

SN754410 Quadruple Half-H Driver IC: Chosen because it has internal isolation between VCC1 (5V logic) and VCC2 (12V motor power) and TTL-compatible inputs for the 3.3V RP4.

IRFZ34 N-Channel MOSFET (Q1): Serves as a master power switch to satisfy the kit component requirement.

Single 4N25 Optocoupler (U1): Provides galvanic isolation for the master enable switch.
Conclusion: The final design successfully meets all project requirements: Bidirectional control, PWM speed control, on/off switching, and use of the kit MOSFET.

</details>

<details>
<summary><strong>Checkpoint 2: AI-Generated Documentation (Click to expand)</strong></summary>

From "Group 5 - CP2 AI Generated Documentation.docx" [cite: Checkpoint2.zip/Group 5 - CP2 AI Generated Documentation.docx]

Conversation #1: Starting Bluetooth Connection

Project Context: Fulfill CP2 requirements to establish a stable, two-way Bluetooth connection between the Android app and the RPi4.

Debugging Log:

Initial Scan Failures: Traced to missing ACCESS_FINE_LOCATION permission, deprecated getParcelableExtra method, and incorrect registerReceiver flags.

App Crash on Connect: Solved a CalledFromWrongThreadException by wrapping all UI updates in runOnUiThread { ... }.

RPi4 "Ghost" Agent: Identified that the Pi's desktop OS was interfering with the connection and showing a pop-up, causing a race condition.

Final Architecture: A team member developed an independent script to handle headless pairing. This simplified the architecture: the RPi4 runs a "dumb" C server (listening on channel 1), and the Android app connects using a reflection hack (createRfcommSocket(1)) to bypass service discovery.

Conversation #2: Headless Bluetooth Pairing & Automation

Project Context: Address the fundamental requirement that the Pi must be "headless" (no user interaction).

Critical Flaw: The Pi's OS (BlueZ) required a manual PIN or confirmation prompt, and retained "stale keys" after a device was "forgotten" on the phone.

Final Design Solution:

clear-bt-devices.sh: A custom script was created to loop through bluetoothctl paired-devices and remove each one.

parmco-server.service: The C server's systemd service was modified to run as root and execute the clear-bt-devices.sh script on boot (ExecStartPre=).

server.c: The C server was modified to call system("/usr/local/bin/clear-bt-devices.sh") on every client disconnect.

Conclusion: This multi-layered solution fully satisfies the "headless" requirement, allowing a user to seamlessly re-pair at any time.

Conversation #3: Motor Control Circuit Update (V10 to Functional)

Critical Issue Discovered: The CP1 circuit design failed. The RP4's GPIO pin sagged to 2.1V and could only source ~4.17mA, which was not enough current to drive the 4N25 optocoupler's LED.

"Tug of War": The 4.7kΩ pull-up resistor (2.55mA pull-HIGH) was 8x stronger than the optocoupler's pull-LOW strength (0.3mA). The MOSFET was stuck ON.

Final Design (V11): The team discovered the optocoupler was unnecessary. The final design removed the optocoupler and used a simple NPN Transistor Buffer (2N3904). The RP4 GPIO now sends a low-current signal to the transistor's base, which reliably controls the MOSFET gate.

</details>

<details>
<summary><strong>Checkpoint 3: AI-Generated Documentation (Click to expand)</strong></summary>

From "Group 5 - CP3 AI Generated Documentation.docx" [cite: Checkpoint3.zip/Group 5 - CP3 AI Generated Documentation.docx]

Project Context: This conversation focused on integrating all remaining motor control features, including the "Automatic" PID feedback mode.
Design Evolution & Debugging Log:

Robust Connection: A "Disconnect" button and a startReadThread() for connection-loss detection were added to MainActivity.kt.

Motor Control Integration: The logic from cp2_keyboard_test.c was merged into server.c. A handle_command() function was created to parse app commands.

RPM Sensing (Pi-to-App Data):

Pi: An ISR (rpm_callback) and a calculator thread (rpm_calculator_thread) were added to server.c to calculate the RPM. A client_write_thread was created to send this data (e.g., "RPM:%.1f\n") to the app.

App: startReadThread() was updated to use a BufferedReader to parse these "RPM:" messages and update the UI.

Closed-Loop Controller (PID Tuning):

V1 (P-Only): Was unstable and oscillated.

V2 (PI-Only): Fixed oscillation but had a large steady-state error (20% offset).

V3 (Advanced User Code): The user provided a new server.c that included a Complementary Filter for sensor smoothing, Gain Scheduling (different PID gains for high/low RPM), and an Error Deadband to prevent oscillation.

V4 (Final PID Tuning): The final iterations involved tuning the user's advanced PID controller to fix the remaining steady-state error by adjusting Kp, Ki, and Kd values.

Compilation Bug: A recurring error: 'rpm_callback' undeclared was fixed by adding a function prototype for rpm_callback at the top of server.c.

Conversation with Claude #1: PID Controller Tuning and Optimization

Initial Problems: The team reported sensor noise, steady-state error at high RPM, and massive phase lag from the initial 5-second moving average filter.

Design Evolution (Filtering):

Exponential Filter (Failed): Introduced too much phase lag.

Median Filter (Failed): Still had too much delay.

Complementary Filter (Success): A new filter (filtered_rpm = 0.85 * instant_rpm + 0.15 * prev_rpm) was implemented with a 10Hz sampling rate, reducing phase lag by 33x.

Final Optimization (Gain Scheduling): To fix a final ±4% oscillation, the team implemented error-based gain scheduling. The controller now uses 4 different Kp multipliers based on how far the current_rpm is from the desired_rpm, allowing it to be aggressive when far away and gentle when close to the target.

Final Performance: The completed controller reaches the target RPM quickly with minimal overshoot and maintains the setpoint accurately across the motor's full 1000-8000 RPM range.

</details>

📁 Final Code Files

<details>
<summary><strong><code>server.c</code> (Raspberry Pi Server) (Click to expand)</strong></summary>

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <pigpio.h>
#include <pthread.h>
#include <time.h>
#include <math.h>

// --- Pin Definitions (BCM) ---
#define MASTER_ENABLE_PIN 17
#define PWM_PIN 18
#define DIR_A_PIN 27
#define DIR_B_PIN 22
#define RPM_SENSOR_PIN 23

// --- PWM Settings ---
#define PWM_FREQUENCY 1000
#define PWM_RANGE 255

// --- RPM CALCULATION ---
#define PULSES_PER_REVOLUTION 3.0
#define RPM_SEND_INTERVAL_MS 250
#define CALCULATION_INTERVAL_MS 100  // Faster sampling for better control (10Hz)

// --- COMPLEMENTARY FILTER (MINIMAL LAG) ---
// Uses instantaneous measurement for fast response, smooths only for noise
#define COMPLEMENTARY_ALPHA 0.85  // 85% new data, 15% smoothing - very responsive!

volatile unsigned long rpm_pulse_count = 0;
volatile double current_rpm = 0.0;
volatile double filtered_rpm = 0.0;
pthread_mutex_t rpm_mutex = PTHREAD_MUTEX_INITIALIZER;

// --- IMPROVED FEEDBACK CONTROLLER ---
#define FEEDBACK_INTERVAL_MS 100
const double feedback_interval_s = 0.1;

// Reduced gains for stability - lower Kp to reduce oscillation
const double Kp_low = 0.0010;   // For RPM < 4000 (reduced from 0.0015)
const double Ki_low = 0.010;    
const double Kd_low = 0.0006;   

const double Kp_high = 0.0014;  // For RPM >= 4000 (reduced from 0.0020)
const double Ki_high = 0.016;   
const double Kd_high = 0.0008;  

const double MAX_INTEGRAL = 100.0;
const double ERROR_DEADBAND = 100.0;  // Increased to absorb ±4% oscillation

// Derivative with higher cutoff (less filtering, more responsive)
#define DERIVATIVE_FILTER_ALPHA 0.8
volatile double filtered_derivative = 0.0;

volatile double integral_error = 0.0;
volatile double previous_error = 0.0;
volatile int reset_integral_flag = 0;
volatile double desired_rpm = 0.0;
volatile int is_automatic_mode = 0;

// --- Global State ---
int current_speed = 0;
int motor_enabled = 0;

// --- Function Prototypes ---
void rpm_callback(int gpio, int level, uint32_t tick);

// --- Helper for milliseconds sleep ---
void sleep_ms(long milliseconds) {
    struct timespec ts;
    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = (milliseconds % 1000) * 1000000;
    nanosleep(&ts, NULL);
}

// --- Motor Control Functions ---
void motor_set_speed(int speed) {
    if (speed < 0) speed = 0;
    if (speed > PWM_RANGE) speed = PWM_RANGE;
    current_speed = speed;
    int duty_cycle = (int)(((float)current_speed / (float)PWM_RANGE) * 1000000);
    gpioHardwarePWM(PWM_PIN, PWM_FREQUENCY, duty_cycle);
    printf("Speed set to %d / 255\n", current_speed);
    fflush(stdout);
}

void motor_quiet_state() {
    is_automatic_mode = 0;
    reset_integral_flag = 1;
    gpioWrite(MASTER_ENABLE_PIN, 1);
    motor_enabled = 0;
    gpioHardwarePWM(PWM_PIN, PWM_FREQUENCY, 0);
    current_speed = 0;
    gpioWrite(DIR_A_PIN, 0);
    gpioWrite(DIR_B_PIN, 0);
    filtered_derivative = 0.0;
    filtered_rpm = 0.0;
    printf("Motor set to quiet state.\n");
    fflush(stdout);
}

void motor_direction_cw() {
    gpioWrite(DIR_A_PIN, 1);
    gpioWrite(DIR_B_PIN, 0);
    printf("Direction: Clockwise\n");
    fflush(stdout);
}

void motor_direction_ccw() {
    gpioWrite(DIR_A_PIN, 0);
    gpioWrite(DIR_B_PIN, 1);
    printf("Direction: Counter-Clockwise\n");
    fflush(stdout);
}

void motor_toggle_power() {
    if (motor_enabled) {
        is_automatic_mode = 0;
        reset_integral_flag = 1;
        gpioWrite(MASTER_ENABLE_PIN, 1);
        motor_enabled = 0;
        printf("MOTOR POWER: OFF\n");
        gpioWrite(DIR_A_PIN, 0);
        gpioWrite(DIR_B_PIN, 0);
        filtered_derivative = 0.0;
        filtered_rpm = 0.0;
    } else {
        gpioWrite(MASTER_ENABLE_PIN, 0);
        motor_enabled = 1;
        printf("MOTOR POWER: ON\n");
        motor_direction_cw();
    }
    fflush(stdout);
}

// --- RPM CALCULATION with COMPLEMENTARY FILTER ---
void* rpm_calculator_thread(void *arg) {
    double calculation_interval_s = CALCULATION_INTERVAL_MS / 1000.0;  // 0.1s
    int first_reading = 1;

    while(1) {
        sleep_ms(CALCULATION_INTERVAL_MS);
        
        // Atomic pulse count reading
        gpioSetISRFunc(RPM_SENSOR_PIN, FALLING_EDGE, 0, NULL);
        unsigned long pulses = rpm_pulse_count;
        rpm_pulse_count = 0;
        gpioSetISRFunc(RPM_SENSOR_PIN, FALLING_EDGE, 0, rpm_callback);

        // Calculate instantaneous RPM
        double revolutions = pulses / PULSES_PER_REVOLUTION;
        double instantaneous_rpm = (revolutions / calculation_interval_s) * 60.0;
        
        pthread_mutex_lock(&rpm_mutex);
        
        if (first_reading) {
            // Initialize filter with first reading
            filtered_rpm = instantaneous_rpm;
            first_reading = 0;
        } else {
            // Complementary filter: mostly new data, slight smoothing
            // This gives ~90% of the responsiveness of raw data with 15% noise reduction
            filtered_rpm = COMPLEMENTARY_ALPHA * instantaneous_rpm + 
                          (1.0 - COMPLEMENTARY_ALPHA) * filtered_rpm;
        }
        
        current_rpm = filtered_rpm;  // Both use filtered value
        
        pthread_mutex_unlock(&rpm_mutex);
    }
    return NULL;
}

// --- RPM ISR Callback ---
void rpm_callback(int gpio, int level, uint32_t tick) {
    rpm_pulse_count++;
}

// --- IMPROVED FEEDBACK CONTROLLER ---
void* feedback_controller_thread(void *arg) {
    double error;
    int adjustment;
    int new_speed;
    double proportional_term, integral_term, derivative_term;
    double Kp, Ki, Kd;
    
    while(1) {
        sleep_ms(FEEDBACK_INTERVAL_MS);

        if (reset_integral_flag) {
            integral_error = 0.0;
            previous_error = 0.0;
            filtered_derivative = 0.0;
            reset_integral_flag = 0;
        }
        
        if (motor_enabled && is_automatic_mode) {
            
            pthread_mutex_lock(&rpm_mutex);
            double rpm_snapshot = current_rpm;
            pthread_mutex_unlock(&rpm_mutex);

            error = desired_rpm - rpm_snapshot;
            
            // Apply deadband to reduce oscillation at steady state
            double error_for_control = error;
            if (fabs(error) < ERROR_DEADBAND) {
                error_for_control = 0.0;
            }
            
            // Adaptive gains based on desired RPM
            if (desired_rpm < 4000.0) {
                Kp = Kp_low;
                Ki = Ki_low;
                Kd = Kd_low;
            } else {
                Kp = Kp_high;
                Ki = Ki_high;
                Kd = Kd_high;
            }
            
            // Proportional term
            proportional_term = Kp * error_for_control;
            
            // Integral term with anti-windup
            if (fabs(error) > ERROR_DEADBAND) {
                integral_error += (error * feedback_interval_s);
                
                // Anti-windup with clamping
                if (integral_error > MAX_INTEGRAL) integral_error = MAX_INTEGRAL;
                if (integral_error < -MAX_INTEGRAL) integral_error = -MAX_INTEGRAL;
            } else {
                // Slowly decay integral when in deadband to prevent buildup
                integral_error *= 0.95;
            }
            integral_term = Ki * integral_error;
            
            // Derivative term with light filtering
            double raw_derivative = (error - previous_error) / feedback_interval_s;
            filtered_derivative = DERIVATIVE_FILTER_ALPHA * raw_derivative + 
                                 (1.0 - DERIVATIVE_FILTER_ALPHA) * filtered_derivative;
            derivative_term = Kd * filtered_derivative;
            
            // Calculate adjustment
            adjustment = (int)(proportional_term + integral_term + derivative_term);
            
            // Rate limiting to prevent violent changes
            if (adjustment > 15) adjustment = 15;
            if (adjustment < -15) adjustment = -15;
            
            new_speed = current_speed + adjustment;
            motor_set_speed(new_speed);
            previous_error = error;
            
            // Debug output every 20 cycles (2 seconds)
            static int debug_counter = 0;
            if (++debug_counter >= 20) {
                printf("[PID] Target: %.1f, Actual: %.1f, Error: %.1f, P: %.2f, I: %.2f, D: %.2f, Adj: %d, Speed: %d\n",
                       desired_rpm, rpm_snapshot, error, proportional_term, integral_term, derivative_term, adjustment, current_speed);
                fflush(stdout);
                debug_counter = 0;
            }
        }
    }
    return NULL;
}

// --- Command Handler ---
void handle_command(char *command) {
    if (strncmp(command, "START_STOP", 10) == 0) motor_toggle_power();
    else if (strncmp(command, "SPEED_UP", 8) == 0) {
        is_automatic_mode = 0;
        reset_integral_flag = 1;
        motor_set_speed(current_speed + 25);
    }
    else if (strncmp(command, "SPEED_DOWN", 10) == 0) {
        is_automatic_mode = 0;
        reset_integral_flag = 1;
        motor_set_speed(current_speed - 25);
    }
    else if (strncmp(command, "DIR_CW", 6) == 0) motor_direction_cw();
    else if (strncmp(command, "DIR_CCW", 7) == 0) motor_direction_ccw();
    else if (strncmp(command, "MODE_AUTO", 9) == 0) {
        is_automatic_mode = 1;
        reset_integral_flag = 1;
        printf("Mode set to AUTO\n"); fflush(stdout);
    }
    else if (strncmp(command, "MODE_MANUAL", 11) == 0) {
        is_automatic_mode = 0;
        reset_integral_flag = 1;
        printf("Mode set to MANUAL\n"); fflush(stdout);
    }
    else if (strncmp(command, "SET_RPM:", 8) == 0) {
        double rpm_from_app = 0.0;
        if (sscanf(command + 8, "%lf", &rpm_from_app) == 1) {
            desired_rpm = rpm_from_app;
            is_automatic_mode = 1;
            reset_integral_flag = 1;
            printf("Desired RPM set to %.2f\n", desired_rpm);
        } else {
            printf("Failed to parse RPM command.\n");
        }
        fflush(stdout);
    }
}

// --- Client Handling Threads ---
typedef struct {
    int client_socket;
    int *is_connected;
} client_thread_args_t;

void* client_read_thread(void *args_ptr) {
    client_thread_args_t *args = (client_thread_args_t*)args_ptr;
    int client = args->client_socket;
    char buf[1024] = { 0 };
    int bytes_read;
    while(1) {
        bytes_read = read(client, buf, sizeof(buf));
        if( bytes_read <= 0 ) break;
        buf[bytes_read] = '\0';
        printf("Received: [%s]", buf);
        fflush(stdout);
        handle_command(buf);
    }
    printf("Client read thread exiting. Setting disconnect flag.\n");
    fflush(stdout);
    *(args->is_connected) = 0;
    motor_quiet_state();
    close(client);
    printf("Client disconnected. Clearing paired devices...\n");
    fflush(stdout);
    system("/usr/local/bin/clear-bt-devices.sh");
    printf("Ready for new connection.\n");
    fflush(stdout);
    free(args);
    return NULL;
}

void* client_write_thread(void *args_ptr) {
    client_thread_args_t *args = (client_thread_args_t*)args_ptr;
    int client = args->client_socket;
    char rpm_buf[32];
    while(*(args->is_connected)) {
        pthread_mutex_lock(&rpm_mutex);
        double rpm = current_rpm;
        pthread_mutex_unlock(&rpm_mutex);
        snprintf(rpm_buf, sizeof(rpm_buf), "RPM:%.1f\n", rpm);
        if (write(client, rpm_buf, strlen(rpm_buf)) < 0) {
            break;
        }
        sleep_ms(RPM_SEND_INTERVAL_MS);
    }
    printf("Client write thread exiting.\n");
    fflush(stdout);
    free(args);
    return NULL;
}

// --- Main Server Function ---
int main(int argc, char **argv)
{
    if (gpioInitialise() < 0) {
        fprintf(stderr, "pigpio initialization failed!\n");
        return 1;
    }

    // Setup GPIO
    gpioSetMode(MASTER_ENABLE_PIN, PI_OUTPUT);
    gpioSetMode(DIR_A_PIN, PI_OUTPUT);
    gpioSetMode(DIR_B_PIN, PI_OUTPUT);
    gpioSetMode(RPM_SENSOR_PIN, PI_INPUT);
    gpioSetPullUpDown(RPM_SENSOR_PIN, PI_PUD_UP);
    gpioSetISRFunc(RPM_SENSOR_PIN, FALLING_EDGE, 0, rpm_callback);
    motor_quiet_state();

    // Start RPM calculator thread
    pthread_t rpm_thread_id;
    if (pthread_create(&rpm_thread_id, NULL, rpm_calculator_thread, NULL)) {
        fprintf(stderr, "Failed to create RPM thread\n");
        gpioTerminate();
        return 1;
    }
    pthread_detach(rpm_thread_id);

    // Start feedback controller thread
    pthread_t feedback_thread_id;
    if (pthread_create(&feedback_thread_id, NULL, feedback_controller_thread, NULL)) {
        fprintf(stderr, "Failed to create feedback thread\n");
        gpioTerminate();
        return 1;
    }
    pthread_detach(feedback_thread_id);

    // --- Bluetooth Setup ---
    struct sockaddr_rc loc_addr = { 0 }, rem_addr = { 0 };
    int s, client;
    socklen_t opt = sizeof(rem_addr);
    printf("Starting RPi4 Bluetooth server...\n");
    fflush(stdout);
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
    loc_addr.rc_family = AF_BLUETOOTH;
    loc_addr.rc_bdaddr = *BDADDR_ANY;
    loc_addr.rc_channel = (uint8_t) 1;
    if (bind(s, (struct sockaddr *)&loc_addr, sizeof(loc_addr)) < 0) {
        perror("Failed to bind socket");
        gpioTerminate();
        return 1;
    }
    listen(s, 1);
    printf("Waiting for connection on RFCOMM channel 1...\n");
    fflush(stdout);

    // --- Main Accept Loop ---
    while(1) {
        client = accept(s, (struct sockaddr *)&rem_addr, &opt);
        if (client < 0) {
            perror("Failed to accept connection");
            continue;
        }
        char buf[32];
        ba2str( &rem_addr.rc_bdaddr, buf );
        fprintf(stdout, "Accepted connection from %s\n", buf);
        fflush(stdout);
        
        // Create client threads
        client_thread_args_t *read_args = malloc(sizeof(client_thread_args_t));
        client_thread_args_t *write_args = malloc(sizeof(client_thread_args_t));
        int *is_connected_flag = malloc(sizeof(int));
        *is_connected_flag = 1;
        read_args->client_socket = client;
        read_args->is_connected = is_connected_flag;
        write_args->client_socket = client;
        write_args->is_connected = is_connected_flag;
        
        pthread_t read_tid, write_tid;
        if (pthread_create(&read_tid, NULL, client_read_thread, (void*)read_args)) {
            perror("Failed to create read thread");
            close(client);
        } else {
            pthread_detach(read_tid);
        }
        if (pthread_create(&write_tid, NULL, client_write_thread, (void*)write_args)) {
            perror("Failed to create write thread");
            close(client);
        } else {
            pthread_detach(write_tid);
        }
    }

    // Cleanup
    close(s);
    motor_quiet_state();
    gpioTerminate();
    return 0;
}


</details>

<details>
<summary><strong><code>MainActivity.kt</code> (Android App) (Click to expand)</strong></summary>

package com.group5.parmco

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothSocket
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.util.Log
import android.view.View
import android.widget.Button
import android.widget.EditText
import android.widget.TextView
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import com.google.android.material.switchmaterial.SwitchMaterial
import java.io.IOException
import java.io.InputStream
import java.io.OutputStream
import java.util.UUID

@SuppressLint("MissingPermission")
class MainActivity : AppCompatActivity() {

    private val tag = "MainActivity"

    companion object {
        private const val RPI_DEVICE_NAME = "group5pi"
        private const val SPP_UUID_STRING = "00001101-0000-1000-8000-00805F9B34FB"
    }

    // --- UI Elements ---
    private lateinit var statusIndicator: View
    private lateinit var statusText: TextView
    private lateinit var connectButton: Button
    private lateinit var connectGroup: View

    // Motor Control UI
    private lateinit var controlsGroup: View
    private lateinit var buttonStartStop: Button
    private lateinit var buttonSpeedDown: Button
    private lateinit var buttonSpeedUp: Button
    private lateinit var switchDirection: SwitchMaterial
    private lateinit var switchMode: SwitchMaterial
    private lateinit var textActualRpm: TextView
    private lateinit var buttonDisconnect: Button
    private lateinit var editDesiredRpm: EditText
    private lateinit var buttonSetRpm: Button
    private lateinit var textDesiredRpm: TextView

    // --- Bluetooth ---
    private var bluetoothAdapter: BluetoothAdapter? = null
    @Volatile private var parmcoSocket: BluetoothSocket? = null
    private var parmcoDevice: BluetoothDevice? = null
    private val sppUuid: UUID = UUID.fromString(SPP_UUID_STRING)
    @Volatile private var outputStream: OutputStream? = null
    private var readThread: Thread? = null

    // --- State ---
    @Volatile private var isMotorRunning = false

    // Permissions
    private val bluetoothPermissions = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
        arrayOf(
            Manifest.permission.BLUETOOTH_SCAN,
            Manifest.permission.BLUETOOTH_CONNECT,
            Manifest.permission.ACCESS_FINE_LOCATION
        )
    } else {
        arrayOf(
            Manifest.permission.ACCESS_FINE_LOCATION
        )
    }

    // --- Activity Result Launchers ---
    private val requestPermissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { permissions ->
            if (permissions.values.all { it }) {
                logToScreen(getString(R.string.log_permissions_granted))
                startConnectionProcess()
            } else {
                logToScreen(getString(R.string.log_error_permission_denied))
                statusIndicator.setBackgroundResource(R.drawable.status_indicator_denied)
            }
        }

    private val requestEnableBluetoothLauncher =
        registerForActivityResult(ActivityResultContracts.StartActivityForResult()) { result ->
            if (result.resultCode == RESULT_OK) {
                logToScreen(getString(R.string.log_bt_enabled))
                startScan()
            } else {
                logToScreen(getString(R.string.log_error_bt_denied))
                statusIndicator.setBackgroundResource(R.drawable.status_indicator_denied)
            }
        }

    // --- Broadcast Receiver ---
    private val scanReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            val action: String? = intent.action
            if (BluetoothDevice.ACTION_FOUND == action) {
                val device: BluetoothDevice? = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                    intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE, BluetoothDevice::class.java)
                } else {
                    @Suppress("DEPRECATION")
                    intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE)
                }
                device?.let {
                    val deviceName = it.name
                    val deviceAddress = it.address
                    if (deviceName != null) {
                        logToScreen(getString(R.string.log_found_device, deviceName, deviceAddress))
                        if (deviceName.equals(RPI_DEVICE_NAME, ignoreCase = true)) {
                            logToScreen(getString(R.string.log_found_rpi))
                            bluetoothAdapter?.cancelDiscovery()
                            parmcoDevice = it
                            connectToDevice()
                        }
                    }
                }
            } else if (BluetoothAdapter.ACTION_DISCOVERY_FINISHED == action) {
                logToScreen(getString(R.string.log_scan_finished))
                if (parmcoDevice == null) {
                    logToScreen(getString(R.string.log_error_rpi_not_found, RPI_DEVICE_NAME))
                    statusIndicator.setBackgroundResource(R.drawable.status_indicator_disconnected)
                }
            }
        }
    }

    // --- Activity Lifecycle ---
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        // Initialize UI elements
        statusIndicator = findViewById(R.id.status_indicator)
        statusText = findViewById(R.id.status_text)
        connectButton = findViewById(R.id.connect_button)
        connectGroup = findViewById(R.id.connect_group)
        controlsGroup = findViewById(R.id.controls_group)
        buttonStartStop = findViewById(R.id.button_start_stop)
        buttonSpeedDown = findViewById(R.id.button_speed_down)
        buttonSpeedUp = findViewById(R.id.button_speed_up)
        switchDirection = findViewById(R.id.switch_direction)
        switchMode = findViewById(R.id.switch_mode)
        textActualRpm = findViewById(R.id.text_actual_rpm)
        buttonDisconnect = findViewById(R.id.button_disconnect)
        editDesiredRpm = findViewById(R.id.edit_desired_rpm)
        buttonSetRpm = findViewById(R.id.button_set_rpm)
        textDesiredRpm = findViewById(R.id.text_desired_rpm)

        // Set initial UI state
        statusText.text = getString(R.string.log_app_started)
        controlsGroup.visibility = View.GONE

        // Get Bluetooth adapter
        val bluetoothManager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        bluetoothAdapter = bluetoothManager.adapter
        if (bluetoothAdapter == null) {
            logToScreen("FATAL: Bluetooth not supported on this device.\n")
            connectButton.isEnabled = false
            return
        }

        // --- Set Click Listeners ---
        connectButton.setOnClickListener {
            logToScreen(getString(R.string.log_connect_clicked))
            parmcoDevice = null
            checkPermissionsAndConnect()
        }
        buttonStartStop.setOnClickListener {
            isMotorRunning = !isMotorRunning
            buttonStartStop.text = if (isMotorRunning) "STOP" else "START"
            sendCommand("START_STOP")
        }
        buttonSpeedUp.setOnClickListener {
            sendCommand("SPEED_UP")
            if (switchMode.isChecked) {
                switchMode.isChecked = false
            }
        }
        buttonSpeedDown.setOnClickListener {
            sendCommand("SPEED_DOWN")
            if (switchMode.isChecked) {
                switchMode.isChecked = false
            }
        }
        switchDirection.setOnCheckedChangeListener { _, isChecked ->
            sendCommand(if (isChecked) "DIR_CCW" else "DIR_CW")
        }
        switchMode.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked) {
                buttonSpeedUp.isEnabled = false
                buttonSpeedDown.isEnabled = false
                sendCommand("MODE_AUTO")
            } else {
                buttonSpeedUp.isEnabled = true
                buttonSpeedDown.isEnabled = true
                sendCommand("MODE_MANUAL")
            }
        }
        buttonDisconnect.setOnClickListener { disconnect() }
        
        buttonSetRpm.setOnClickListener {
            val rpmString = editDesiredRpm.text.toString()
            if (rpmString.isNotEmpty()) {
                sendCommand("SET_RPM:$rpmString")
                textDesiredRpm.text = rpmString
                if (!switchMode.isChecked) {
                    switchMode.isChecked = true
                }
            } else {
                logToScreen("Please enter a desired RPM.\n")
            }
        }

        // Register the broadcast receiver
        val filter = IntentFilter(BluetoothDevice.ACTION_FOUND)
        filter.addAction(BluetoothAdapter.ACTION_DISCOVERY_FINISHED)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            registerReceiver(scanReceiver, filter, RECEIVER_EXPORTED)
        } else {
            @Suppress("DEPRECATION")
            registerReceiver(scanReceiver, filter)
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        unregisterReceiver(scanReceiver)
        disconnect()
    }

    private fun logToScreen(message: String) {
        statusText.append(message)
        Log.d(tag, message)
    }

    // --- Permission and Scan Logic ---
    private fun checkPermissionsAndConnect() {
        val permissionsToRequest = bluetoothPermissions.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }
        if (permissionsToRequest.isEmpty()) {
            logToScreen(getString(R.string.log_permissions_good))
            startConnectionProcess()
        } else {
            val permsString = permissionsToRequest.joinToString()
            logToScreen(getString(R.string.log_requesting_permissions, permsString))
            requestPermissionLauncher.launch(permissionsToRequest.toTypedArray())
        }
    }
    private fun startConnectionProcess() {
        if (bluetoothAdapter?.isEnabled == false) {
            logToScreen(getString(R.string.log_bt_off))
            val enableBtIntent = Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)
            requestEnableBluetoothLauncher.launch(enableBtIntent)
        } else {
            logToScreen(getString(R.string.log_bt_on))
            startScan()
        }
    }
    private fun startScan() {
        if (bluetoothAdapter?.isDiscovering == true) {
            bluetoothAdapter?.cancelDiscovery()
        }
        logToScreen(getString(R.string.log_scan_starting))
        statusIndicator.setBackgroundResource(R.drawable.status_indicator_connecting)
        bluetoothAdapter?.startDiscovery()
    }

    // --- Command/Disconnect Logic ---
    private fun sendCommand(command: String) {
        val outStream = outputStream
        if (outStream == null) {
            logToScreen("ERROR: Not connected. Cannot send command.\n")
            return
        }
        Thread {
            try {
                outStream.write("$command\n".toByteArray())
                runOnUiThread { logToScreen("Sent: $command\n") }
            } catch (e: IOException) {
                Log.e(tag, "Error sending command", e)
                disconnect()
            }
        }.start()
    }

    private fun disconnect() {
        if (parmcoSocket == null) return
        val socket = parmcoSocket
        val outStream = outputStream
        parmcoSocket = null
        outputStream = null
        isMotorRunning = false

        try {
            readThread?.interrupt()
            outStream?.close()
            socket?.close()
        } catch (e: IOException) {
            Log.e(tag, "Error closing socket", e)
        }
        
        runOnUiThread {
            logToScreen("Disconnected.\n")
            statusIndicator.setBackgroundResource(R.drawable.status_indicator_denied)
            controlsGroup.visibility = View.GONE
            connectGroup.visibility = View.VISIBLE
            buttonStartStop.text = "START"
            textActualRpm.text = "--"
            textDesiredRpm.text = "--"
            
            switchMode.isChecked = false
            buttonSpeedUp.isEnabled = true
            buttonSpeedDown.isEnabled = true
        }
    }

    // --- Read Thread ---
    private fun startReadThread() {
        readThread = Thread {
            val inputStream = parmcoSocket?.inputStream
            try {
                inputStream?.bufferedReader()?.use { reader ->
                    while (parmcoSocket != null && !Thread.currentThread().isInterrupted) {
                        val line = reader.readLine()
                        if (line == null) {
                            break
                        }
                        if (line.startsWith("RPM:")) {
                            val rpmValue = line.substringAfter("RPM:").trim()
                            runOnUiThread {
                                textActualRpm.text = rpmValue
                            }
                        }
                    }
                }
            } catch (e: IOException) {
                Log.w(tag, "Connection lost (read failed): ${e.message}")
            }
            disconnect()
        }
        readThread?.start()
    }


    // --- Connection Logic (Reflection Hack) ---
    private fun connectToDevice() {
        Thread {
            try {
                runOnUiThread {
                    logToScreen(getString(R.string.log_creating_socket))
                }
                val method = parmcoDevice?.javaClass?.getMethod(
                    "createRfcommSocket",
                    Int::class.javaPrimitiveType
                )
                parmcoSocket = method?.invoke(parmcoDevice, 1) as BluetoothSocket
                parmcoSocket?.connect()
                outputStream = parmcoSocket?.outputStream
                startReadThread()
                runOnUiThread {
                    logToScreen(getString(R.string.log_connected_success))
                    statusIndicator.setBackgroundResource(R.drawable.status_indicator_connected)
                    controlsGroup.visibility = View.VISIBLE
                    connectGroup.visibility = View.GONE
                }
            } catch (e: Exception) {
                val errorMsg = e.message ?: "Unknown Exception"
                runOnUiThread {
                    logToScreen(getString(R.string.log_error_connection_failed, errorMsg))
                    statusIndicator.setBackgroundResource(R.drawable.status_indicator_denied)
                }
                try {
                    parmcoSocket?.close()
                } catch (closeException: IOException) {
                    val closeMsg = closeException.message ?: "Unknown close exception"
                    runOnUiThread {
                        logToScreen(getString(R.string.log_error_socket_close, closeMsg))
                    }
                }
            }
        }.start()
    }
}


</details>

<details>
<summary><strong><code>activity_main.xml</code> (Android App Layout) (Click to expand)</strong></summary>

<?xml version="1.0" encoding="utf-8"?>
<androidx.constraintlayout.widget.ConstraintLayout
    xmlns:android="[http://schemas.android.com/apk/res/android](http://schemas.android.com/apk/res/android)"
    xmlns:app="[http://schemas.android.com/apk/res-auto](http://schemas.android.com/apk/res-auto)"
    xmlns:tools="[http://schemas.android.com/tools](http://schemas.android.com/tools)"
    android:layout_width="match_parent"
    android:layout_height="match_parent"
    tools:context=".MainActivity">

    <!-- Status Indicator -->
    <View
        android:id="@+id/status_indicator"
        android:layout_width="32dp"
        android:layout_height="32dp"
        android:layout_marginTop="24dp"
        android:background="@drawable/status_indicator_disconnected"
        app:layout_constraintEnd_toEndOf="parent"
        app:layout_constraintStart_toStartOf="parent"
        app:layout_constraintTop_toTopOf="parent" />

    <!-- Guideline -->
    <androidx.constraintlayout.widget.Guideline
        android:id="@+id/horizontal_guideline"
        android:layout_width="wrap_content"
        android:layout_height="wrap_content"
        android:orientation="horizontal"
        app:layout_constraintGuide_percent="0.65" />

    <!-- Connection Group -->
    <androidx.constraintlayout.widget.ConstraintLayout
        android:id="@+id/connect_group"
        android:layout_width="0dp"
        android:layout_height="wrap_content"
        android:layout_marginStart="16dp"
        android:layout_marginEnd="16dp"
        app:layout_constraintBottom_toTopOf="@+id/horizontal_guideline"
        app:layout_constraintEnd_toEndOf="parent"
        app:layout_constraintStart_toStartOf="parent"
        app:layout_constraintTop_toBottomOf="@+id/status_indicator"
        app:layout_constraintVertical_bias="0.5">

        <Button
            android:id="@+id/connect_button"
            android:layout_width="0dp"
            android:layout_height="wrap_content"
            android:text="@string/connect_button_text"
            app:layout_constraintBottom_toBottomOf="parent"
            app:layout_constraintEnd_toEndOf="parent"
            app:layout_constraintStart_toStartOf="parent"
            app:layout_constraintTop_toTopOf="parent" />

    </androidx.constraintlayout.widget.ConstraintLayout>

    <!-- Motor Controls Group -->
    <androidx.constraintlayout.widget.ConstraintLayout
        android:id="@+id/controls_group"
        android:layout_width="0dp"
        android:layout_height="0dp"
        android:layout_marginStart="16dp"
        android:layout_marginEnd="16dp"
        android:layout_marginTop="8dp"
        android:layout_marginBottom="8dp"
        android:visibility="gone"
        app:layout_constraintBottom_toTopOf="@+id/horizontal_guideline"
        app:layout_constraintEnd_toEndOf="parent"
        app:layout_constraintStart_toStartOf="parent"
        app:layout_constraintTop_toBottomOf="@+id/status_indicator"
        tools:visibility="visible">

        <!-- Buttons: Start, Speed -->
        <Button
            android:id="@+id/button_start_stop"
            android:layout_width="0dp"
            android:layout_height="wrap_content"
            android:text="START"
            app:layout_constraintEnd_toEndOf="parent"
            app:layout_constraintStart_toStartOf="parent"
            app:layout_constraintTop_toTopOf="parent" />

        <Button
            android:id="@+id/button_speed_down"
            android:layout_width="0dp"
            android:layout_height="wrap_content"
            android:layout_marginTop="8dp"
            android:layout_marginEnd="4dp"
            android:text="Speed -"
            app:layout_constraintEnd_toStartOf="@+id/button_speed_up"
            app:layout_constraintHorizontal_bias="0.5"
            app:layout_constraintStart_toStartOf="parent"
            app:layout_constraintTop_toBottomOf="@+id/button_start_stop" />

        <Button
            android:id="@+id/button_speed_up"
            android:layout_width="0dp"
            android:layout_height="wrap_content"
            android:layout_marginStart="4dp"
            android:text="Speed +"
            app:layout_constraintEnd_toEndOf="parent"
            app:layout_constraintHorizontal_bias="0.5"
            app:layout_constraintStart_toEndOf="@+id/button_speed_down"
            app:layout_constraintTop_toTopOf="@+id/button_speed_down" />

        <!-- Switches: Direction, Mode -->
        <com.google.android.material.switchmaterial.SwitchMaterial
            android:id="@+id/switch_direction"
            android:layout_width="0dp"
            android:layout_height="wrap_content"
            android:layout_marginTop="8dp"
            android:text="Direction (OFF=CW, ON=CCW)"
            android:textSize="16sp"
            app:layout_constraintEnd_toEndOf="parent"
            app:layout_constraintStart_toStartOf="parent"
            app:layout_constraintTop_toBottomOf="@+id/button_speed_down" />

        <com.google.android.material.switchmaterial.SwitchMaterial
            android:id="@+id/switch_mode"
            android:layout_width="0dp"
            android:layout_height="wrap_content"
            android:layout_marginTop="8dp"
            android:text="Mode (OFF=Manual, ON=Auto)"
            android:textSize="16sp"
            app:layout_constraintEnd_toEndOf="parent"
            app:layout_constraintStart_toStartOf="parent"
            app:layout_constraintTop_toBottomOf="@+id/switch_direction" />

        <!-- RPM Display -->
        <TextView
            android:id="@+id/label_actual_rpm"
            android:layout_width="wrap_content"
            android:layout_height="wrap_content"
            android:layout_marginTop="16dp"
            android:text="Actual RPM: "
            android:textSize="18sp"
            android:textStyle="bold"
            app:layout_constraintStart_toStartOf="parent"
            app:layout_constraintTop_toBottomOf="@+id/switch_mode" />

        <TextView
            android:id="@+id/text_actual_rpm"
            android:layout_width="wrap_content"
            android:layout_height="wrap_content"
            android:text="--"
            android:textSize="18sp"
            app:layout_constraintBottom_toBottomOf="@+id/label_actual_rpm"
            app:layout_constraintStart_toEndOf="@+id/label_actual_rpm"
            app:layout_constraintTop_toTopOf="@+id/label_actual_rpm" />

        <!-- NEW: Desired RPM Display -->
        <TextView
            android:id="@+id/label_desired_rpm"
            android:layout_width="wrap_content"
            android:layout_height="wrap_content"
            android:layout_marginTop="8dp"
            android:text="Desired RPM: "
            android:textSize="18sp"
            android:textStyle="bold"
            app:layout_constraintStart_toStartOf="parent"
            app:layout_constraintTop_toBottomOf="@+id/label_actual_rpm" />

        <TextView
            android:id="@+id/text_desired_rpm"
            android:layout_width="wrap_content"
            android:layout_height="wrap_content"
            android:text="--"
            android:textSize="18sp"
            app:layout_constraintBottom_toBottomOf="@+id/label_desired_rpm"
            app:layout_constraintStart_toEndOf="@+id/label_desired_rpm"
            app:layout_constraintTop_toTopOf="@+id/label_desired_rpm" />
        
        <!-- Desired RPM Input -->
        <EditText
            android:id="@+id/edit_desired_rpm"
            android:layout_width="0dp"
            android:layout_height="wrap_content"
            android:layout_marginTop="8dp"
            android:layout_marginEnd="8dp"
            android:hint="Set Target RPM"
            android:inputType="number"
            app:layout_constraintEnd_toStartOf="@+id/button_set_rpm"
            app:layout_constraintStart_toStartOf="parent"
            app:layout_constraintTop_toBottomOf="@+id/label_desired_rpm" />

        <Button
            android:id="@+id/button_set_rpm"
            android:layout_width="wrap_content"
            android:layout_height="wrap_content"
            android:text="Set"
            app:layout_constraintBottom_toBottomOf="@+id/edit_desired_rpm"
            app:layout_constraintEnd_toEndOf="parent"
            app:layout_constraintTop_toTopOf="@+id/edit_desired_rpm" />

        <!-- Disconnect Button -->
        <Button
            android:id="@+id/button_disconnect"
            android:layout_width="0dp"
            android:layout_height="wrap_content"
            android:layout_marginTop="8dp"
            android:text="@string/disconnect_button_text"
            app:layout_constraintEnd_toEndOf="parent"
            app:layout_constraintStart_toStartOf="parent"
            app:layout_constraintTop_toBottomOf="@+id/edit_desired_rpm" />

    </androidx.constraintlayout.widget.ConstraintLayout>

    <!-- Log Scroller -->
    <ScrollView
        android:id="@+id/log_scroller"
        android:layout_width="0dp"
        android:layout_height="0dp"
        android:layout_margin="16dp"
        android:background="#ECECEC"
        android:padding="8dp"
        app:layout_constraintBottom_toBottomOf="parent"
        app:layout_constraintEnd_toEndOf="parent"
        app:layout_constraintStart_toStartOf="parent"
        app:layout_constraintTop_toBottomOf="@+id/horizontal_guideline">

        <TextView
            android:id="@+id/status_text"
            android:layout_width="match_parent"
            android:layout_height="wrap_content"
            android:fontFamily="monospace"
            android:textColor="#000000"
            android:textSize="12sp"
            tools:text="Log will appear here..." />

    </ScrollView>

</androidx.constraintlayout.widget.ConstraintLayout>


</details>
