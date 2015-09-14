/*********************************************************************************************************
/ Aaron Dawson - C Programming Final Project
/ Impulse Response Generator "Impulsinator"
/ This program creates an impulse response representative of a given room with various parameters
/ and also displays a visual representation of the ray tracing process
/
/
/
*********************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <sndfile.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <ncurses.h>
#include <stdbool.h>
#include <math.h>
#include <ctype.h>

// OpenGL
#ifdef __MACOSX_CORE__
  #include <GLUT/glut.h>
#else
  #include <GL/gl.h>
  #include <GL/glu.h>
  #include <GL/glut.h>
#endif

// General parameters
#define SAMPLE_RATE 44100 // in Hz
#define STEREO 2
#define MONO 1
#define NUM_OUT_CHANNELS STEREO
#define SPEED_OF_SOUND 343 // in meters per second
#define INCHES_PER_METER 39.37
#define OUTFILE_LENGTH_IN_SECONDS 10 // duration of output WAV file
#define NUM_RAYS 10000 // number of rays to be used in the computation
#define LENGTH SAMPLE_RATE * OUTFILE_LENGTH_IN_SECONDS // the length represented in samples
#define GENERATE_IMPULSE false // if true, compute the impulse response and write to WAV file
#define SHOW_VISUAL true // if true, show visual simulation of ray tracing

// Source location parameters
#define SOURCE_LOCATION_X 0.0 // in meters
#define SOURCE_LOCATION_Y 0.0 // in meters
#define SOURCE_LOCATION_Z 0.0 // in meters

// Room parameters (X = width, Y = height, Z = depth)
#define ROOM_CENTER_X 0.0 // in meters
#define ROOM_CENTER_Y 0.0 // in meters
#define ROOM_CENTER_Z 0.0 // in meters
#define ROOM_WIDTH_X 5 // in meters
#define ROOM_WIDTH_Y 6 // in meters
#define ROOM_WIDTH_Z 4 // in meters
#define ROOM_ABSORPTION_COEFF 0.99 // scales the amplitude by this amount when a ray collides with a wall
#define LOWPASS_COEFF 0.90 /* filters the sound using a simple lowpass filter with the difference equation: 
							y[n] = x[n] + ax[n-1] where n is the sample number, y is the output signal, 
							x is the input signal, and a is equal to LOWPASS_COEFF */
#define FILTERING true // filtering is only included when this is set to true

// Microphone parameters
#define MIC_LOCATION_X 0.0 // in meters
#define MIC_LOCATION_Y 1.0 // in meters
#define MIC_LOCATION_Z 0.2 // in meters
#define MIC2_LOCATION_X 0 // in meters
#define MIC2_LOCATION_Y 1.0 // in meters
#define MIC2_LOCATION_Z -0.2 // in meters
#define MIC_WIDTH 1.5/INCHES_PER_METER // in inches; the program assumes both mics are the same size

// OpenGL parameters
#define PRINT_RAY_INFO false // determines whether or not to print when each ray is received
#define SCALE 400 // scaling factor used to determine initial size of objects drawn on screen
#define INIT_WIDTH 1000 // initial width of the window
#define INIT_HEIGHT 900 // initial height of the window
#define INIT_ANGLE_X 0 // initial angle of rotation about the x axis
#define INIT_ANGLE_Y -15 // initial angle of rotation about the y axis
#define POINT_SIZE 1.0 // how large the points are drawn
#define COLOR_INCR 0.05 // used to change the color of the room when the 'c' key is pressed
#define ROTATION_INCR 0.10f // increment by which the view is rotated
#define ZOOM_AMT 1.001f // value used in calculations for zooming in and out
#define NUM_SIM_RAYS 100000 /* the number of rays that will be drawn on the screen (independent of the actual
							rays used in the impulse response simulation) */
#define SHOW_ABSORPTION true /* if true is selected, lines will get less bright each time they bounce off
								of a wall, representing their loss of energy during the collision. */	
#define RAY_SPEED 1 // the speed at which rays move across the screen
#define FADE_AMT 10 /* from 1 to 10; as FADE_AMT increases the ray's tail will fade out quicker. Only 
						implemented when DRAW_MODE LINES is in effect. */
#define RAY_SPEED_INCR 2 // increment used to change the speed of the rays on the screen
#define RAY_TAIL_LENGTH 1000 // Do not change this from 1000!
#define POINTS 0 // view rays as points
#define LINES 1 // view rays as lines
#define DRAW_MODE POINTS /* DRAW_MODE refers to how the ray will be drawn. If POINTS is selected, it
						will be drawn as a point that moves across the screen. If LINES is selected, it
						will be drawn as a line with tail length = RAY_TAIL_LENGTH (kind of like
						snake).*/

// Print options
#define PRINT_RAY false // determines whether information about each ray is printed when it is received
#define PRINT_INFO true // determines whether timing information and number of rays received is printed

// Global variables
float g_room_width_x = ROOM_WIDTH_X;
float g_room_width_y = ROOM_WIDTH_Y;
float g_room_width_z = ROOM_WIDTH_Z;

float g_mic_location_x = MIC_LOCATION_X;
float g_mic_location_y = MIC_LOCATION_Y;
float g_mic_location_z = MIC_LOCATION_Z;
float g_mic2_location_x = MIC2_LOCATION_X;
float g_mic2_location_y = MIC2_LOCATION_Y;
float g_mic2_location_z = MIC2_LOCATION_Z;

// libsndfile
SNDFILE *outfile;
SF_INFO sfinfo_out;

// calculate the distance (in meters) that sound will travel during one sampling interval
float distance_per_sampling_interval = (float) SPEED_OF_SOUND / (float) SAMPLE_RATE;

// for counting total number of rays received
int numRaysReceived = 0;

// struct containing room parameters
typedef struct {
	float x_min, x_max, y_min, y_max, z_min, z_max;	
	float color1, color2, color3;
} room;

// struct containing ray parameters
typedef struct {
	float x, y, z, x_inc, y_inc, z_inc;
	float color1, color2, color3;
	bool received;
	float amplitude; 
	int numReflections;
} ray;

// Global pointer to the room
room *g_roomPtr;

// Translation
bool g_translate = false;

// Rotation
bool g_key_rotate_x = false;
bool g_key_rotate_y = false;
GLfloat g_inc_y = 0.0;
GLfloat g_inc_x = 0.0;
GLfloat scale = 1.0;
GLfloat g_rotationAmt = 0.1f;
static int fastRotate = 1;

// Scaling
bool g_key_zoomIn = false;
bool g_key_zoomOut = false;

// Start simulation
bool g_start = false;

/************************************************************************
GLOBAL VARIABLES FOR OpenGL
************************************************************************/
// width and height of the window
GLsizei g_width = INIT_WIDTH;
GLsizei g_height = INIT_HEIGHT;
GLsizei g_last_width = INIT_WIDTH;
GLsizei g_last_height = INIT_HEIGHT;

static float g_ray_speed = RAY_SPEED; // initial speed of ray
static int g_draw_mode = DRAW_MODE; // drawing mode

GLint g_colorDirection1 = 1; // 1 = increasing; -1 = decreasing.
GLint g_colorDirection2 = 1; // 1 = increasing; -1 = decreasing.
GLint g_colorDirection3 = 1; // 1 = increasing; -1 = decreasing.

// fill mode
GLenum g_fillmode = GL_FILL;

// fullscreen
GLboolean g_fullscreen = false;

// modelview stuff
GLfloat g_linewidth = 1.0f;

// array of rays to be used in the visual simulation
static ray g_rays[NUM_SIM_RAYS];

// array of pointers to the rays used in the simulation
ray *g_rayPtrs[NUM_SIM_RAYS];

// buffers used to store past locations of each ray (in order to create the tail effect)
GLfloat *g_buffer_x[NUM_SIM_RAYS];
GLfloat *g_buffer_y[NUM_SIM_RAYS];
GLfloat *g_buffer_z[NUM_SIM_RAYS];

//-----------------------------------------------------------------------------
// function prototypes
//-----------------------------------------------------------------------------
bool setup_output_file();
void initializeRay(ray *);
void propagateRay(ray *);
void checkBoundaries(ray *, room *);
void checkMicrophone(ray *, float *, float *, int, int);
void checkMicrophone_Visual(ray *, int);
void writeToWav(float *, float *, SNDFILE *);
void initializeRoom(room *);
double calculate(struct rusage *, struct rusage *);
bool checkInitVals();
void idleFunc( );
void displayFunc( );
void reshapeFunc( int width, int height );
void keyboardFunc( unsigned char, int, int );
void initialize_graphics();
void initialize_glut(int argc, char *argv[]);
void drawRoom(room *);
void rotateView();
void keyboardUpFunc(unsigned char, int, int);
void drawEdge(float, float, float, float, float, float, float, room *);
void drawRay(ray *, int);
void drawSoundSource();
void drawMicrophones();
float power(float, int);
void printInstructions();
float getScale();
void scaleView();
void printComputationTime(float, char *);
void choosePreset(int);
/***************************************************************************/
int main(int argc, char **argv) {
	int i, j;

	// If two arguments are specified
	if (argc == 2) {
		for (i=0; i<4;i++) {
			// if the argument corresponds with a preset
			if (atoi(argv[1]) == i) {
				// load that preset	
				choosePreset(i);
			}
		}
	}
	
	// seed random number generator
	srand(time(NULL));

	// Make sure all #define values make sense
	if (!checkInitVals()) {
		printf("Computation aborted.\n");
		return EXIT_FAILURE;
	}

	// create and initialize room
	static room room1;
	room *roomPtr = &room1;
	g_roomPtr = &room1;
	initializeRoom(roomPtr);

	// If the user wants to generate an impulse and write it to a WAV file
	if (GENERATE_IMPULSE) {
		// structs for timing data
		struct rusage before, after, start, end;
		// get start time
		getrusage(RUSAGE_SELF, &before);
		// return if file cannot be opened
		if (!setup_output_file()) {
			return EXIT_FAILURE;
		}
	
		// create array of floats for output
		float *out = (float *)malloc(sizeof(float)*LENGTH); // left (mic1)
		float *out2 = (float *)malloc(sizeof(float)*LENGTH); // right (mic2)
	
		// fill outfile with zeros
		for(i = 0; i < LENGTH; i++) {
			out[i] = 0.0f;
		}
		initscr();
		static double totalRayDuration = 0;
		// for each ray
		for(i = 0; i < NUM_RAYS; i++) {	
			// get start time
			getrusage(RUSAGE_SELF, &start);
			// create a new ray
			static ray ray1;		
			// create pointer to ray
			ray *rayPtr = &ray1;
			// initialize ray
			initializeRay(rayPtr);		
			for (j = 0; j < LENGTH; j++) {
				// if the ray has not reached the microphone
				if (rayPtr->received == false) {
					// propogate the ray
					propagateRay(rayPtr);
					// check boundaries
					checkBoundaries(rayPtr, roomPtr);	
					// check mic
					checkMicrophone(rayPtr, out, out2, j, i);				
				}
				else {
					break; // move to the next ray once the current one is received
				}
			}
			// get end time
			getrusage(RUSAGE_SELF, &end);
			double rayDuration = calculate(&start, &end);
			totalRayDuration += rayDuration;
			clear();
			printw("The simulation is %.3f percent complete (%d of %d rays computed).\n",i*100/(float)NUM_RAYS,i,NUM_RAYS);
			printComputationTime((totalRayDuration/i)*(NUM_RAYS - i),"remaining");
			printComputationTime(totalRayDuration,"elapsed");
//			printw("Remaining time: %f.\n", (totalRayDuration/i)*(NUM_RAYS - i));
			refresh();
		}
		endwin();
		// write output to wav
		writeToWav(out, out2, outfile);
	
		if (PRINT_INFO) {
			// get end time
			getrusage(RUSAGE_SELF, &after);
			// calculate duration
			double duration = calculate(&before, &after);
			// print duration
			printf("Using %d rays, your impulse response was created in %f seconds.\n", NUM_RAYS, duration);
			printf("In total, %d rays were received by the virtual microphone.\n", numRaysReceived);
		}
	}

	// If the user wants to see a visual representation of the ray tracing
	if (SHOW_VISUAL) {
		// Initialize Glut
		initialize_glut(argc, argv);
	
		// set each pointer to its respective ray
		for (i=0;i<NUM_SIM_RAYS;i++) {
			g_rayPtrs[i] = &g_rays[i];	
		}
		
		// allocate memory for buffers
		for (i=0;i<NUM_SIM_RAYS;i++) {
			g_buffer_x[i] = (GLfloat *)malloc(RAY_TAIL_LENGTH*sizeof(GLfloat));
			g_buffer_y[i] = (GLfloat *)malloc(RAY_TAIL_LENGTH*sizeof(GLfloat));
			g_buffer_z[i] = (GLfloat *)malloc(RAY_TAIL_LENGTH*sizeof(GLfloat));
		}
	
		// initialize the rays to be used in the simulation
		for (i=0; i<NUM_SIM_RAYS; i++) {
			initializeRay(g_rayPtrs[i]);
		}
	
		// draw the room
		drawRoom(roomPtr);
	
		// instructions for the visual simulation
		printInstructions();

		// repeats until 'q' is pressed
		glutMainLoop();
	}
	
	return 0;
}
/***************************************************************************/
// Sets up the output file
bool setup_output_file() {
	
	memset(&sfinfo_out, 0, sizeof(SF_INFO));
	sfinfo_out.samplerate = SAMPLE_RATE;
	sfinfo_out.channels = NUM_OUT_CHANNELS;
	sfinfo_out.format = SF_FORMAT_WAV | SF_FORMAT_FLOAT;
	if (!sf_format_check(&sfinfo_out)) {
		printf("Error: Incorrect audio file format");
		return false;
	}

	if (( outfile = sf_open( "impulse.wav", SFM_WRITE, &sfinfo_out ) ) == NULL ) {
		printf("Error, couldn't open the file\n");
		return false;
	}

	return true;
}

// Sets up initial values for a ray
void initializeRay(ray *rayPtr) {
		// determine starting location for each ray
		// Set initial ray position
		rayPtr->x = SOURCE_LOCATION_X;
		rayPtr->y = SOURCE_LOCATION_Y;
		rayPtr->z = SOURCE_LOCATION_Z;
	
		// establish that each ray has not yet reached the microphone
		rayPtr->received = false;

		// set initial amplitude and number of reflections for each ray
		int random1 = rand()%2;
		float coeff;

		// Randomize the phase of each ray
		if (random1 == 0) {
			coeff = -1.0;
		}
		else {
			coeff = 1.0;
		}
		rayPtr->amplitude = coeff;
		rayPtr->numReflections = 0;

		// set the color (values only used for visual simulation)
		rayPtr->color1 = ((float)rand()/(float)RAND_MAX);
		rayPtr->color2 = ((float)rand()/(float)RAND_MAX);
		rayPtr->color3 = ((float)rand()/(float)RAND_MAX);

		// Generate initial vector for ray
		rayPtr->x_inc = ((float)rand()/(float)RAND_MAX - 0.5f)*0.02;
		rayPtr->y_inc = ((float)rand()/(float)RAND_MAX - 0.5f)*0.02;
		rayPtr->z_inc = ((float)rand()/(float)RAND_MAX - 0.5f)*0.02;
	
		// compute magnitude
		float magnitude = sqrt(rayPtr->x_inc * rayPtr->x_inc + rayPtr->y_inc * 
			rayPtr->y_inc + rayPtr->z_inc * rayPtr->z_inc);

		// normalize the vector
		rayPtr->x_inc *= distance_per_sampling_interval/magnitude;
		rayPtr->y_inc *= distance_per_sampling_interval/magnitude;
		rayPtr->z_inc *= distance_per_sampling_interval/magnitude;
}

// Propagates the ray
void propagateRay(ray *rayPtr) {
	// move the ray to its new position
	rayPtr->x += rayPtr->x_inc;
	rayPtr->y += rayPtr->y_inc;
	rayPtr->z += rayPtr->z_inc;
}

// Check to see if the ray has hit a wall
void checkBoundaries(ray *rayPtr, room *room1) {	
	// if the ray has surpassed the right wall
	if (rayPtr->x >= room1->x_max) {
		rayPtr->x_inc = -rayPtr->x_inc;
		rayPtr->x += rayPtr->x_inc;
		rayPtr->numReflections++;
//		printf("The right wall was hit!\n");
	}
	// if the ray has surpassed the left wall
	else if (rayPtr->x <= room1->x_min) {
		rayPtr->x_inc = -rayPtr->x_inc;
		rayPtr->x += rayPtr->x_inc;	
		rayPtr->numReflections++;
//		printf("The left wall was hit!\n");
	}
	// if the ray has surpassed the ceiling
	else if (rayPtr->y >= room1->y_max) {
		rayPtr->y_inc = -rayPtr->y_inc;
		rayPtr->y += rayPtr->y_inc;
		rayPtr->numReflections++;
//		printf("The ceiling was hit!\n");
	}
	// if the ray has surpassed the floor
	else if (rayPtr->y <= room1->y_min) {
		rayPtr->y_inc = -rayPtr->y_inc;
		rayPtr->y += rayPtr->y_inc;
		rayPtr->numReflections++;
//		printf("The floor was hit!\n");
	}
	// if the ray has surpassed the front wall
	else if (rayPtr->z >= room1->z_max) {
		rayPtr->z_inc = -rayPtr->z_inc;
		rayPtr->z += rayPtr->z_inc;
		rayPtr->numReflections++;
//		printf("The front wall was hit!\n");
	}
	// if the ray has surpassed the back wall
	else if (rayPtr->z <= room1->z_min) {
		rayPtr->z_inc = -rayPtr->z_inc;	
		rayPtr->z += rayPtr->z_inc;
		rayPtr->numReflections++;
//		printf("The back wall was hit!\n");
	}
}

// Check the microphone to see if the ray has been received
void checkMicrophone(ray *rayPtr, float *out, float *out2, int j, int i) {

	int k;
	
	//If the ray has been received by the left mic
	if (   rayPtr->x > g_mic_location_x - MIC_WIDTH && rayPtr->x < g_mic_location_x + MIC_WIDTH
		&& rayPtr->y > g_mic_location_y - MIC_WIDTH && rayPtr->y < g_mic_location_y + MIC_WIDTH
		&& rayPtr->z > g_mic_location_z - MIC_WIDTH && rayPtr->z < g_mic_location_z + MIC_WIDTH) {
		
		// scale the amplitude based on how many times it reflected and the absorption coefficient
		rayPtr->amplitude *= power(ROOM_ABSORPTION_COEFF, rayPtr->numReflections);

		// print each ray received, if specified
		if (PRINT_RAY) {
			printf("Ray #%d has been received by mic #1. This ray has reflected %d times.\n", i, rayPtr->numReflections);
		}
		// if filtering is enabled
		if (FILTERING) {
			// loop to add ray to the output buffer (including filtering)
			for (k = 0; k < rayPtr->numReflections; k++) {			
				// as long as the current sample position (j) plus the delay (k) is less than the buffer length
				if (j+k <= LENGTH) {
					// record ray in output buffer
					out[j+k] += rayPtr->amplitude;
					// scale the amplitude by the lowpass coefficient
					rayPtr->amplitude *= LOWPASS_COEFF;
				}
			}
		}
		// if filtering is not enabled
		else {
			out[j] = rayPtr->amplitude;
		}
		
		// stop the ray's propagation
		rayPtr->received = true;

		// update total number of rays received
		numRaysReceived++;
	}
	//If the ray has been received by the right mic
	else if (   rayPtr->x > g_mic2_location_x - MIC_WIDTH && rayPtr->x < g_mic2_location_x + MIC_WIDTH
		&& rayPtr->y > g_mic2_location_y - MIC_WIDTH && rayPtr->y < g_mic2_location_y + MIC_WIDTH
		&& rayPtr->z > g_mic2_location_z - MIC_WIDTH && rayPtr->z < g_mic2_location_z + MIC_WIDTH) {
		
		// scale the amplitude based on how many times it reflected and the absorption coefficient
		rayPtr->amplitude *= power(ROOM_ABSORPTION_COEFF, rayPtr->numReflections);

		// print each ray received, if specified
		if (PRINT_RAY) {
			printf("Ray #%d has been received by mic #2. This ray has reflected %d times.\n", i, rayPtr->numReflections);
		}
		// if filtering is enabled
		if (FILTERING) {
			// loop to add ray to the output buffer (including filtering)
			for (k = 0; k < rayPtr->numReflections; k++) {			
				// as long as the current sample position (j) plus the delay (k) is less than the buffer length
				if (j+k <= LENGTH) {
					// record ray in output buffer
					out2[j+k] += rayPtr->amplitude;
					// scale the amplitude by the lowpass coefficient
					rayPtr->amplitude *= LOWPASS_COEFF;
				}
			}
		}
		// if filtering is not enabled
		else {
			out2[j] = rayPtr->amplitude;
		}
		
		// stop the ray's propagation
		rayPtr->received = true;

		// update total number of rays received
		numRaysReceived++;
	}
}

// Check to see if a ray has been received (used for the visual simulation)
void checkMicrophone_Visual(ray *rayPtr, int j) {
	//If the ray has been received by the left mic
	if (   rayPtr->x > g_mic_location_x - MIC_WIDTH && rayPtr->x < g_mic_location_x + MIC_WIDTH
		&& rayPtr->y > g_mic_location_y - MIC_WIDTH && rayPtr->y < g_mic_location_y + MIC_WIDTH
		&& rayPtr->z > g_mic_location_z - MIC_WIDTH && rayPtr->z < g_mic_location_z + MIC_WIDTH) 
	{
	// stop the ray's propagation
		rayPtr->received = true;
		if (PRINT_RAY_INFO) {
			printf("Ray #%d has been received by the left microphone after %d reflections!\n", j, rayPtr->numReflections);
		}
	}
	//If the ray has been received by the right mic
	else if (   rayPtr->x > g_mic2_location_x - MIC_WIDTH && rayPtr->x < g_mic2_location_x + MIC_WIDTH
		&& rayPtr->y > g_mic2_location_y - MIC_WIDTH && rayPtr->y < g_mic2_location_y + MIC_WIDTH
		&& rayPtr->z > g_mic2_location_z - MIC_WIDTH && rayPtr->z < g_mic2_location_z + MIC_WIDTH)
	{
	// stop the ray's propagation
		rayPtr->received = true;
		if (PRINT_RAY_INFO) {
			printf("Ray #%d has been received by the right microphone after %d reflections!\n", j, rayPtr->numReflections);
		}
	}
}

// Write output to WAV
void writeToWav(float *out, float *out2, SNDFILE *outfile) {
	float max = 0;
	int i;

	// for output received by left mic
	for (i=0; i < LENGTH; i++) {
		// if the sample amplitude is greater than max
		if (fabs(out[i]) > max) {
			// set the max equal to this value
			max = fabs(out[i]);
		}
	}
	// for output received by right mic
	for (i=0; i < LENGTH; i++) {
		// if the sample amplitude is greater than max
		if (fabs(out2[i]) > max) {
			// set the max equal to this value
			max = fabs(out2[i]);
		}
	}

	// divide each sample by maximum value to normalize
	for (i=0; i < LENGTH; i++) {
		out[i] /= max;
		out2[i] /= max;
	}

	// create new buffer for stereo output
	float *stereo_out = (float *)malloc(sizeof(float)*LENGTH*2); 
	
	// fill stereo buffer with signals from both mics
	for (i=0; i < LENGTH; i++) {
		stereo_out[2*i] = out[i]; // left mic
		stereo_out[2*i+1] = out2[i]; // right mic
	}
	
	// write to WAV file
	sf_writef_float(outfile, stereo_out, LENGTH);

	// close file
	sf_close(outfile);

}

// Initialize room parameters
void initializeRoom(room *roomPtr) {	
	// set values for min and max
	roomPtr->x_min = ROOM_CENTER_X - g_room_width_x/2;
	roomPtr->x_max = roomPtr->x_min + g_room_width_x;
	roomPtr->y_min = ROOM_CENTER_Y - g_room_width_y/2;
	roomPtr->y_max = roomPtr->y_min + g_room_width_y;
	roomPtr->z_min = ROOM_CENTER_Z - g_room_width_z/2;
	roomPtr->z_max = roomPtr->z_min + g_room_width_z;

	// set room color
	roomPtr->color1 = ((float)rand()/(float)RAND_MAX);
	roomPtr->color2 = ((float)rand()/(float)RAND_MAX);
	roomPtr->color3 = ((float)rand()/(float)RAND_MAX);
}

/* Returns number of seconds between b and a. */
double calculate(struct rusage *b, struct rusage *a)
{
    if (b == NULL || a == NULL)
        return 0;
    else
        return ((((a->ru_utime.tv_sec * 1000000 + a->ru_utime.tv_usec) -
                        (b->ru_utime.tv_sec * 1000000 + b->ru_utime.tv_usec)) +
                    ((a->ru_stime.tv_sec * 1000000 + a->ru_stime.tv_usec) -
                     (b->ru_stime.tv_sec * 1000000 + b->ru_stime.tv_usec))) 
                / 1000000.);
}

/* Calculates the remaining computation time required. */
void printComputationTime(float seconds, char *string) {
	int minutes = (int)seconds/60;
	int hours = minutes/60;
	char *s = "s", *s1 = "s", *s2 = "s";
	if (seconds < 60) {
		if ((int)seconds == 1) {
			s = "";
		} else {
			s = "s";
		}
		printw("Time %s: %d second%s.\n", string, (int)seconds, s);
	} else if (seconds < 3600) {
		seconds -= 60*minutes;
		if ((int)seconds == 1) {
			s = "";
		} else {
			s = "s";
		}
		if (minutes == 1) {
			s1 = "";
		} else {
			s1 = "s";
		}
		printw("Time %s: %d minute%s and %d second%s.\n", string, minutes, s1, (int)seconds, s);
	} else if (seconds < 86400) {
		seconds -= 60*minutes;
		minutes -= 60*hours;
		if ((int)seconds == 1) {
			s = "";
		} else {
			s = "s";
		}
		if (minutes == 1) {
			s1 = "";
		} else {
			s1 = "s";
		}
		if (hours == 1) {
			s2 = "";
		} else {
			s2 = "s";
		}
		printw("Time %s: %d hour%s, %d minute%s, and %d second%s.\n", string, hours, s2, minutes, s1, (int)seconds, s);
	}
}

/* Checks to make sure sound source, room, and microphone are set up properly */
bool checkInitVals() {
	bool proceed = true;
	// If the sound source is outside the room
	if (SOURCE_LOCATION_X > ROOM_CENTER_X + g_room_width_x/2 || 
		SOURCE_LOCATION_Y > ROOM_CENTER_Y + g_room_width_y/2 ||
		SOURCE_LOCATION_Z > ROOM_CENTER_Z + g_room_width_z/2) {
		printf("Make sure the source is located inside the room.\n");
		proceed = false;
	}
	// If microphone 1 is outside the room
	if (g_mic_location_x > ROOM_CENTER_X + g_room_width_x/2 || 
		g_mic_location_y > ROOM_CENTER_Y + g_room_width_y/2 ||
		g_mic_location_z > ROOM_CENTER_Z + g_room_width_z/2) {
		printf("Make sure microphone 1 is located inside the room.\n");
		proceed = false;
	}
	// If microphone 2 is outside the room
	if (g_mic2_location_x > ROOM_CENTER_X + g_room_width_x/2 || 
		g_mic2_location_y > ROOM_CENTER_Y + g_room_width_y/2 ||
		g_mic2_location_z > ROOM_CENTER_Z + g_room_width_z/2) {
		printf("Make sure microphone 2 is located inside the room.\n");
		proceed = false;
	}

	// If the sound source is located inside the microphone
	if (fabs(SOURCE_LOCATION_X - g_mic_location_x) <= MIC_WIDTH/2 && 
		fabs(SOURCE_LOCATION_Y - g_mic_location_y) <= MIC_WIDTH/2 &&
		fabs(SOURCE_LOCATION_Z - g_mic_location_z) <= MIC_WIDTH/2) {
		printf("Make sure the sound source is located at a sufficient distance from mic 1.\n");
		proceed = false;
	}
	// If the sound source is located inside the microphone
	if (fabs(SOURCE_LOCATION_X - g_mic2_location_x) <= MIC_WIDTH/2 && 
		fabs(SOURCE_LOCATION_Y - g_mic2_location_y) <= MIC_WIDTH/2 &&
		fabs(SOURCE_LOCATION_Z - g_mic2_location_z) <= MIC_WIDTH/2) {
		printf("Make sure the sound source is located at a sufficient distance from mic 2.\n");
		proceed = false;
	}
	// If draw mode is not properly set
	if (DRAW_MODE != 0 && DRAW_MODE != 1) {
		printf("DRAW_MODE was set to %d. It must be set to either 0 or 1.\n", DRAW_MODE);
		proceed = false;
	}
	// If fade amount is not properly set
	if (FADE_AMT >  10) {
		printf("FADE_AMT was set to %d. It cannot be greater than 10.\n", FADE_AMT);
		proceed = false;
	}
	// If ray tail length is not properly set
	if (RAY_TAIL_LENGTH != 1000) {
		printf("RAY_TAIL_LENGTH was set to %d. It must be 1000, otherwise the program won't work.\n", RAY_TAIL_LENGTH);
		proceed = false;
	}
	return proceed;
}

//-----------------------------------------------------------------------------
// Name: initialize_glut( )
// Desc: Initializes Glut with the global vars
//-----------------------------------------------------------------------------
void initialize_glut(int argc, char *argv[]) {
    // initialize GLUT
    glutInit( &argc, argv );
    // double buffer, use rgb color, enable depth buffer
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH );
    // initialize the window size
    glutInitWindowSize( g_width, g_height );
    // set the window postion
    glutInitWindowPosition( 400, 100 );
    // create the window
    glutCreateWindow( "ImpulseGenerator");
    // full screen
    if ( g_fullscreen ) glutFullScreen();
    // set the idle function - called when idle
    glutIdleFunc( idleFunc );
    // set the display function - called when redrawing
    glutDisplayFunc( displayFunc );
    // set the reshape function - called when client area changes
    glutReshapeFunc( reshapeFunc );
    // set the keyboard function - called on keyboard events
    glutKeyboardFunc( keyboardFunc );
	// set the keyboard function - called on keyboard events
    glutKeyboardUpFunc( keyboardUpFunc );
    // do our own initialization
    initialize_graphics( );  
}

//-----------------------------------------------------------------------------
// Name: initialize_graphics( )
// Desc: sets initial OpenGL states and initializes any application data
//-----------------------------------------------------------------------------
void initialize_graphics()
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);                 // Black Background
    // enable depth
    glEnable( GL_DEPTH_TEST );
    // set fill mode
    glPolygonMode( GL_FRONT_AND_BACK, g_fillmode );
    // line width
    glLineWidth( g_linewidth );
	// for round points
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

//-----------------------------------------------------------------------------
// Name: reshapeFunc( )
// Desc: called when window size changes
//-----------------------------------------------------------------------------
void reshapeFunc( int w, int h )
{
    // save the new window size
    g_width = w; g_height = h;
    // set the matrix mode to project
    glMatrixMode( GL_PROJECTION );
    // load the identity matrix
    glLoadIdentity( );
    // map the view port to the client area
    glViewport( 0, 0, w, h );
    // create the viewing frustum
    GLfloat fovy = 45.0f;
    GLfloat zNear = .05f;
    GLfloat zFar = 7500.0f;
    gluPerspective( fovy, (GLfloat) w / (GLfloat) h, zNear, zFar);
    // set the matrix mode to modelview
    glMatrixMode( GL_MODELVIEW );
    // load the identity matrix
    glLoadIdentity( );

    // position the view point
    //  void gluLookAt( GLdouble eyeX,
    //                 GLdouble eyeY,
    //                 GLdouble eyeZ,
    //                 GLdouble centerX,
    //                 GLdouble centerY,
    //                 GLdouble centerZ,
    //                 GLdouble upX,
    //                 GLdouble upY,
    //                 GLdouble upZ )

    // gluLookAt( 0.0f, 0.0f, 30.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f );
    gluLookAt( 0.0f, 0.0f, h / 2.0f / tanf(fovy/2.0f), 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f );

}

//-----------------------------------------------------------------------------
// Name: idleFunc( )
// Desc: callback from GLUT
//-----------------------------------------------------------------------------
void idleFunc( )
{
    // render the scene
    glutPostRedisplay( );
}

//-----------------------------------------------------------------------------
// Name: keyboardFunc( )
// Desc: key event
//-----------------------------------------------------------------------------
void keyboardFunc( unsigned char key, int x, int y )
{
	int i,j;

	float scale = getScale();

	/* calculate the value with which to scale the rotation amount. This is necessary
		because the rotateView() function is called every time a ray is scaled or rotated,
		so the rotation increment must be scaled to compensate for changes in the number
		of rays or the speed of the rays (which is implemented by updating the rays position
		more quickly). */
	float scaleBy = g_rotationAmt/((float)NUM_SIM_RAYS*g_ray_speed);
	if (scaleBy < 0.00005) {
		scaleBy = 0.00005;
	}
		
    //printf("key: %c\n", key);
    switch( key )
    {
		case '1':
			g_start = true;
			break;
		case ' ':
			fastRotate = 4;
			break;
		case 'x':
			g_key_zoomIn = true; // zooms in
			break;
		case 'z':
			g_key_zoomOut = true; // zooms out
			break;
		case '=':
			g_ray_speed *= RAY_SPEED_INCR; // increases ray propagation speed
			break;
		case '-':
			if (g_ray_speed >= 1) {
				g_ray_speed /= RAY_SPEED_INCR; // decreases ray propagation speed
			}
			break;
		case 'c': // change color of room
			if(g_roomPtr->color1+COLOR_INCR*2*g_colorDirection1 < 1 && g_roomPtr->color1+COLOR_INCR*2*g_colorDirection1 > 0) {
				g_roomPtr->color1 += COLOR_INCR * g_colorDirection1;
			} else {
				g_colorDirection1 *= -1;
				g_roomPtr->color1 += COLOR_INCR * g_colorDirection1;
			}
			if(g_roomPtr->color2+COLOR_INCR*2*g_colorDirection2 < 1 && g_roomPtr->color2+COLOR_INCR*2*g_colorDirection2 > 0) {
				g_roomPtr->color2 += COLOR_INCR * 0.83 * g_colorDirection2;
			} else {
				g_colorDirection2 *= -1;
				g_roomPtr->color2 += COLOR_INCR * 0.83 * g_colorDirection2;
			}
			if(g_roomPtr->color3+COLOR_INCR*2*g_colorDirection3 < 1 && g_roomPtr->color3+COLOR_INCR*2*g_colorDirection3 > 0) {
				g_roomPtr->color3 += COLOR_INCR * 0.73 * g_colorDirection3;
			} else {
				g_colorDirection3 *= -1;
				g_roomPtr->color3 += COLOR_INCR * 0.73 * g_colorDirection3;
			}

			break;
		case 'm': // toggle between draw modes 0 and 1 (points and line, respectively)
			for (i=0;i<NUM_SIM_RAYS;i++) {
				for(j=0;j<RAY_TAIL_LENGTH;j++) {
					g_buffer_x[i][j] = g_rayPtrs[i]->x*scale;
					g_buffer_y[i][j] = g_rayPtrs[i]->y*scale;
					g_buffer_z[i][j] = g_rayPtrs[i]->z*scale;
				}
			}
			if (g_draw_mode == 1) {
				g_draw_mode--;
			} else {
				g_draw_mode++;
			}
						// Fullscreen
			break;
        case 'f':
            if( !g_fullscreen )
            {
                g_last_width = g_width;
                g_last_height = g_height;
                glutFullScreen();
            }
            else
                glutReshapeWindow( g_last_width, g_last_height );
				
            g_fullscreen = !g_fullscreen;
            printf("[ImpulseResponse]: fullscreen: %s\n", g_fullscreen ? "ON" : "OFF" );
            break;

        case 'q':
            // Exit window
			exit( 0 );
            break;
	    case 'a': // Arrow key left is pressed
			g_inc_y = -scaleBy*(float)fastRotate;		
            g_key_rotate_y = true;
            break;
        case 'd':    // Arrow key right is pressed
			g_inc_y = scaleBy*(float)fastRotate;
            g_key_rotate_y = true;
            break;
        case 'w' :        // Arrow key up is pressed
			g_inc_x = scaleBy*(float)fastRotate;
            g_key_rotate_x = true;
            break;
        case 's' :    // Arrow key down is pressed
			g_inc_x = -scaleBy*(float)fastRotate;
	        g_key_rotate_x = true;
            break; 
		case 'p':
			if (g_start) {
				g_start = false;
			} else {
				g_start = true;
			}
			break;
		}
}

// used to stop the rotation when a key is released
void keyboardUpFunc(unsigned char key, int x, int y ) {
	switch(key) {
		case 'x':
			g_key_zoomIn = false;
			break;
		case 'z':
			g_key_zoomOut = false;
			break;
		case 'a':
			g_key_rotate_y = false;
			break;
		case 'd':
			g_key_rotate_y = false;
			break;
		case 'w':
			g_key_rotate_x = false;
			break;
		case 's':
			g_key_rotate_x = false;
			break;
		case ' ':
			fastRotate = 1;
		}
}

//-----------------------------------------------------------------------------
// Name: displayFunc( )
// Desc: callback function invoked to draw the client area
//-----------------------------------------------------------------------------
void displayFunc( )
{ 
	int i,j;
    // clear the color and depth buffers
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	drawRoom(g_roomPtr);
	drawSoundSource();
	drawMicrophones();
	for (i=0;i<g_ray_speed;i++) {
		for(j=0;j<NUM_SIM_RAYS;j++) {
			drawRay(g_rayPtrs[j],j);
		}
	}	
    // flush gl commands
    glFlush( );

    // swap the buffers
    glutSwapBuffers( );
}

//-----------------------------------------------------------------------------
// Name: void rotateView ()
// Desc: Rotates the current view by the angle specified in the globals
//-----------------------------------------------------------------------------
void rotateView () {
    static GLfloat angle_x = INIT_ANGLE_X;
    static GLfloat angle_y = INIT_ANGLE_Y;

    if (g_key_rotate_y) {
        glRotatef ( angle_y += g_inc_y, 0.0f, 1.0f, 0.0f );
    }
    else {
        glRotatef (angle_y, 0.0f, 1.0f, 0.0f );
    }

    if (g_key_rotate_x) {
        glRotatef ( angle_x += g_inc_x, 1.0f, 0.0f, 0.0f );
    }
    else {
        glRotatef (angle_x, 1.0f, 0.0f, 0.0f );
    }
}

// scales the current view
void scaleView () {
	static GLfloat scaled_x = 1.0f;
	static GLfloat scaled_y = 1.0f;
	static GLfloat scaled_z = 1.0f;

	/* scale factor used to compensate for the effects of NUM_SIM_RAYS and 
		g_ray_speed on the zoom rate */
	float scaleBy = pow(ZOOM_AMT, 1.0/((float)NUM_SIM_RAYS*g_ray_speed/10));
	if (scaleBy < 1.000001) {
		scaleBy = 1.000001;
	}
	// zoom in
	if (g_key_zoomIn) {
		scaled_x *= scaleBy;
		scaled_y *= scaleBy;
		scaled_z *= scaleBy;
		glScalef(scaled_x, scaled_y, scaled_z);
	}
	// zoom out
	else if (g_key_zoomOut) {
		scaled_x *= 1.0f/scaleBy;
		scaled_y *= 1.0f/scaleBy;
		scaled_z *= 1.0f/scaleBy;
		glScalef(scaled_x, scaled_y, scaled_z);
	}
	else {
		glScalef(scaled_x, scaled_y, scaled_z);
	}
}

// Draws one edge of the room (a line connecting two vertices)
void drawEdge(float x, float y, float z, float x2, float y2, float z2, float scale, room *roomPtr) {

	glPushMatrix();
	{
		rotateView();
		scaleView();
		glBegin(GL_LINES);
		glColor4f(fmod(roomPtr->color1+(float)COLOR_INCR,1), fmod(roomPtr->color2+(float)COLOR_INCR,1), fmod(roomPtr->color3+(float)COLOR_INCR,1), 1);
		glVertex3f(x*scale,y*scale,z*scale);
		glVertex3f(x2*scale,y2*scale,z2*scale);
		glEnd();
	}
	glPopMatrix();

}

// Draws the sound source as a point
void drawSoundSource() {

	glPointSize(POINT_SIZE*4);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glPushMatrix();
	{
		rotateView();
		scaleView();
		glBegin(GL_POINTS);
		glColor4f(0.2, 0.7, 0.5, 1);
		glVertex3f(SOURCE_LOCATION_X, SOURCE_LOCATION_Y, SOURCE_LOCATION_Z);
		glEnd();
	}
	glPopMatrix();
}

// Draws the room by drawing each of the edges
void drawRoom(room *roomPtr) {

	float scale = getScale();
	// Draw the edges of the room
	drawEdge(roomPtr->x_min, roomPtr->y_min, roomPtr->z_min,
			roomPtr->x_max, roomPtr->y_min, roomPtr->z_min, scale, roomPtr);

	drawEdge(roomPtr->x_min, roomPtr->y_min, roomPtr->z_min,
			roomPtr->x_min, roomPtr->y_min, roomPtr->z_max, scale, roomPtr);

	drawEdge(roomPtr->x_min, roomPtr->y_min, roomPtr->z_min,
			roomPtr->x_min, roomPtr->y_max, roomPtr->z_min, scale, roomPtr);

	drawEdge(roomPtr->x_min, roomPtr->y_max, roomPtr->z_min,
			roomPtr->x_min, roomPtr->y_max, roomPtr->z_max, scale, roomPtr);

	drawEdge(roomPtr->x_min, roomPtr->y_max, roomPtr->z_min,
			roomPtr->x_max, roomPtr->y_max, roomPtr->z_min, scale, roomPtr);

	drawEdge(roomPtr->x_min, roomPtr->y_max, roomPtr->z_max,
			roomPtr->x_min, roomPtr->y_min, roomPtr->z_max, scale, roomPtr);

	drawEdge(roomPtr->x_min, roomPtr->y_max, roomPtr->z_max,
			roomPtr->x_max, roomPtr->y_max, roomPtr->z_max, scale, roomPtr);

	drawEdge(roomPtr->x_max, roomPtr->y_min, roomPtr->z_max,
			roomPtr->x_min, roomPtr->y_min, roomPtr->z_max, scale, roomPtr);

	drawEdge(roomPtr->x_max, roomPtr->y_min, roomPtr->z_max,
			roomPtr->x_max, roomPtr->y_min, roomPtr->z_min, scale, roomPtr);

	drawEdge(roomPtr->x_max, roomPtr->y_min, roomPtr->z_max,
			roomPtr->x_max, roomPtr->y_max, roomPtr->z_max, scale, roomPtr);

	drawEdge(roomPtr->x_max, roomPtr->y_max, roomPtr->z_min,
			roomPtr->x_max, roomPtr->y_min, roomPtr->z_min, scale, roomPtr);

	drawEdge(roomPtr->x_max, roomPtr->y_max, roomPtr->z_min,
			roomPtr->x_max, roomPtr->y_max, roomPtr->z_max, scale, roomPtr);

}

// Draws the microphones as points
void drawMicrophones() {

	float scale = getScale();
	glPointSize(POINT_SIZE*4);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glPushMatrix();
	{
		rotateView();
		scaleView();
		glBegin(GL_POINTS);
		glColor4f(0.9, 0.1, 0.4, 1);
		glVertex3f(g_mic_location_x*scale, g_mic_location_y*scale, g_mic_location_z*scale);
		glEnd();
	}
	glPopMatrix();

	glPushMatrix();
	{
		rotateView();
		scaleView();
		glBegin(GL_POINTS);
		glColor4f(0.9, 0.1, 0.4, 1);
		glVertex3f(g_mic2_location_x*scale, g_mic2_location_y*scale, g_mic2_location_z*scale);
		glEnd();
	}
	glPopMatrix();

}

// Draws a ray either as a point (if draw mode is 0) or as a line (if draw mode is 1)
void drawRay(ray *rayPtr, int j) {
	int i;

	float scale = getScale();
	
	// used to decrease opacity over time if absorption is included as a simulation parameter
	static float absorption = 1;

	// If point mode is selected
	if (g_draw_mode == 0) {

		glPointSize(POINT_SIZE);
		glEnable(GL_POINT_SMOOTH);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
		glPushMatrix();
		{
			rotateView();
			scaleView();
			glBegin(GL_POINTS);
			if (SHOW_ABSORPTION) {
				absorption = power((float)ROOM_ABSORPTION_COEFF, rayPtr->numReflections);
			}
			glColor4f(rayPtr->color1, rayPtr->color2, rayPtr->color3, 1*absorption);
			
			glVertex3f(rayPtr->x*scale, rayPtr->y*scale, rayPtr->z*scale);
			if (rayPtr->received == false && g_start) {
				propagateRay(rayPtr);
				checkBoundaries(rayPtr, g_roomPtr);
				checkMicrophone_Visual(rayPtr, j);
			}
			glEnd();
		}
		glPopMatrix();
	}
	// If line mode is selected
	else if (g_draw_mode == 1) {
		// update the buffers, propagate the ray
		if (rayPtr->received == false && g_start) {
			// shift the values in each array back one position
			for (i=0; i < RAY_TAIL_LENGTH; i++) {
				g_buffer_x[j][RAY_TAIL_LENGTH-i] = g_buffer_x[j][RAY_TAIL_LENGTH-i-1];
				g_buffer_y[j][RAY_TAIL_LENGTH-i] = g_buffer_y[j][RAY_TAIL_LENGTH-i-1];
				g_buffer_z[j][RAY_TAIL_LENGTH-i] = g_buffer_z[j][RAY_TAIL_LENGTH-i-1];
			}
			g_buffer_x[j][0] = rayPtr->x*scale; // the first position of each array is the current location
			g_buffer_y[j][0] = rayPtr->y*scale;
			g_buffer_z[j][0] = rayPtr->z*scale;
			propagateRay(rayPtr);
			checkBoundaries(rayPtr, g_roomPtr);
			checkMicrophone_Visual(rayPtr, j);
		}
		// draw the ray
		glPushMatrix();
		{
			rotateView();
			scaleView();
			glBegin(GL_LINE_STRIP);
			for (i=0;i<RAY_TAIL_LENGTH;i++) {
				float scaleFactor = (float) power((float)(RAY_TAIL_LENGTH-i), FADE_AMT)/power((float)RAY_TAIL_LENGTH, FADE_AMT);
				if (SHOW_ABSORPTION) {
					absorption = power((float)ROOM_ABSORPTION_COEFF, rayPtr->numReflections);
				}
	
				glColor4f(rayPtr->color1, rayPtr->color2, rayPtr->color3, scaleFactor*absorption);				
				glLineWidth(7*scaleFactor);

				glVertex3f(g_buffer_x[j][i], g_buffer_y[j][i], g_buffer_z[j][i]);
			}
			glEnd();
		}
		glPopMatrix();
	}
	
}

// calculates the power given a base and exponent
float power(float base, int pow) {
	if (pow == 0) {
		return 1;
	} else if (pow == 1) {
		return base;
	}
	return base * power(base, pow-1);
}

// prints instructions for the visual simulation
void printInstructions()
{
    printf( "----------------------------------------------------\n" );
    printf( "Impulse Response Simulator\n" );
    printf( "by Aaron Dawson, 2014\n" );
    printf( "New York University\n" );
    printf( "----------------------------------------------------\n" );
    printf( "'1' - start simulation\n" );
    printf( "'f' - toggle fullscreen\n" );
    printf( "'=/-' - change ray propagation speed\n" );
    printf( "'z/x' - zoom\n" );
	printf( "'a/d/w/s' - rotate view\n" );
	printf( "'spacebar' - faster rotation\n" );
	printf( "'c' - change room color\n" );
	printf( "'m' - change draw mode\n" );
	printf( "'p' - pause\n" );
    printf( "'q' - quit\n" );
    printf( "----------------------------------------------------\n" );
    printf( "\n" );
}

/* Function to get scaling factor for use in setting default sizes. Thus,
	the room will always be initialized to appropriate size on the screen
	regardless of what its actual size is. In other words, a room that is 
	3x4x5 will appear just as large as a room that is 30x40x50, and all other
	objects on the screen will be scaled accordingly. */
float getScale() {
	// determine scaling factor
	float max = g_room_width_x;
	if (g_room_width_y > max) {
		max = g_room_width_y;
	}
	if (g_room_width_z > max) {
		max = g_room_width_z;
	}
	float scale = (float)SCALE/max;

	return scale;
}

// initializes a preset
void choosePreset(int i) {
	switch ( i )
	{
		case 0:
			g_room_width_x = 0.5;
			g_room_width_y = 0.4;
			g_room_width_z = 0.3;
			g_mic_location_x = 0.2;
			g_mic_location_y = 0.03;
			g_mic_location_z = 0.02;
			g_mic2_location_x = -0.2;
			g_mic2_location_y = 0.03;
			g_mic2_location_z = 0.02;

			break;
		case 1:
			g_room_width_x = 10;
			g_room_width_y = 1.5;
			g_room_width_z = 2;
			break;
		case 2:
			g_room_width_x = 50;
			g_room_width_y = 20;
			g_room_width_z = 30;
			g_mic_location_x = 20;
			g_mic_location_y = 0.0;
			g_mic_location_z = 0.2;
			g_mic2_location_x = 20;
			g_mic2_location_y = 0.0;
			g_mic2_location_z = -0.2;
			break;
		case 3:
			g_room_width_x = 10;
			g_room_width_y = 3;
			g_room_width_z = 15;
			break;
	}
}
