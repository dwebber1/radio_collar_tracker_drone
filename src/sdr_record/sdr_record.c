/*
RCT Payload Software
Copyright (C) 2016  Hui, Nathan T.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/**
 * This program records broadband signal data from an RTL2832 based SDR to
 * assist detection of radio collars.
 * Copyright (C) 2015 by Nathan Hui <nthui@eng.ucsd.edu>
 */
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <rtl-sdr.h>
#include <signal.h>
#include <pthread.h>
#include <sys/queue.h>
#include <time.h>
#include <stdbool.h>
#include "queue.h"
#include <stdint.h>
#include <libairspy/airspy.h>
#include <libairspy/airspy_commands.h>

// Global constants
#define FILE_CAPTURE_DAEMON_SLEEP_PERIOD_MS	50
#define FRAMES_PER_FILE	80
#define META_PREFIX "META_"


enum airspy_sample_type sample_type_val = AIRSPY_SAMPLE_INT16_IQ;

#define INT16_EL_SIZE_BYTE (2) 

// Typedefs
struct proc_queue_args {
	int run_num;
	int frame_len;
};

// Global variables
rtlsdr_dev_t *dev = NULL;
//pointer for airspy
struct airspy_device* device = NULL;
volatile int run = 1;
pthread_mutex_t logid ck;
queue data_queue;
int counter = 0;
static uint64_t num_samples = 0;
char* DATA_DIR = "/home/vm/EFE/";

// Function Prototypes
void sighandler(int signal);
int main(int argc, char** argv);
void* proc_queue(void* args);
int airspy_callback(airspy_transfer_t* transfer);
//static void rtlsdr_callback(unsigned char* buf, uint32_t len, void *ctx);
void lock_mutex();
void printUsage();

/**
 * Prints the usage statement for this program.
 */
void printUsage() {
	printf("usage: sdr_record -r <run_number> [-h] [-g <gain>]\n\t"
	       "[-f <center_frequency>] [-s <sampling_frequency>] [-o <output_dir>]\n");
}




/**
 * Main function of this program.
 */
int main(int argc, char** argv) {
	// Set up varaibles
	// Options variable
	int opt;
	// SDR Gain
	int gain = 0;
	// Sampling Frequency for SDR in samples per second
	int samp_freq = 2048000;
	// Center Frequency for SDR in Hz
	int center_freq = 172464000;
	// Run number
	int run_num = -1;
	// Block size
	int block_size = 0;
	// Start Time
	struct timespec start_time;
	// Thread attributes
	pthread_attr_t attr;
	// Thread ID
	pthread_t thread_id;
	// proc_queue args
	struct proc_queue_args pargs;
	// Device ID
	int dev_id = 0;
	//result for airspy
	int result;
	//airspy_samplefrequency 
	int air_samp_freq =2500000;
	// Get command line options
	// printf("Getting command line options\n");
	while ((opt = getopt(argc, argv, "hg:s:f:r:o:d:")) != -1) {
		result = AIRSPY_SUCCESS;
		switch (opt) {
			case 'h':
				printUsage();
				exit(0);
			case 'g':
				gain = (int)(atof(optarg) * 10);
				break;
			case 's':
				samp_freq = (int)(atof(optarg));
				break;
			case 'f':
				center_freq = (int)(atof(optarg));
				break;
			case 'r':
				run_num = (int)(atof(optarg));
				break;
			case 'o':
				DATA_DIR = optarg;
				break;		
			
			default:
				printf("unknown argument '-%c %s'\n", opt, optarg);
				
				return EXIT_FAILURE;
		}

		if( result != AIRSPY_SUCCESS ) {
			printf("argument error: '-%c %s' %s (%d)\n", opt, optarg, airspy_error_name(result), result);
			usage();
			return EXIT_FAILURE;
		}	

	


	}
	if (run_num == -1) {
		// TODO: add usage notification here!
		fprintf(stderr, "ERROR: Bad RUN number!\n");
		printUsage();
		exit(-1);
	}
	printf("SDR_RECORD: Run Number %d\n", run_num);
	// printf("done\n");

	// Initialize environment
	// printf("Configuring environment\n");
	block_size = 262144;
	queue_init(&data_queue);
	// printf("Done configuring environment\n");

	// Open SDR
	printf("Opening SDR\n");
	if (rtlsdr_open(&dev, dev_id)) {
		fprintf(stderr, "ERROR: Failed to open rtlsdr device\n");
		exit(1);
	}
	printf("SDR Opened\n");

	result = airspy_init(); //init sdr
	if( result != AIRSPY_SUCCESS ) {
		printf("airspy_init() failed: %s (%d)\n", airspy_error_name(result), result);
		return EXIT_FAILURE;
	}

	result = airspy_open(&device);
	if(result !=AIRSPY_SUCCESS){
		printf("airspy_open() failed: %s (%d)\n", airspy_error_name(result), result);
			airspy_exit();
			return EXIT_FAILURE;
	}
	result = airspy_set_sample_type(device, sample_type_val);
	if (result != AIRSPY_SUCCESS) {
		printf("airspy_set_sample_type() failed: %s (%d)\n", airspy_error_name(result), result);
		airspy_close(device);
		airspy_exit();
		return EXIT_FAILURE;
	}


	result = airspy_set_samplerate(device, air_samp_freq);
	if (result != AIRSPY_SUCCESS) {
		printf("airspy_set_samplerate() failed: %s (%d)\n", airspy_error_name(result), result);
		airspy_close(device);
		airspy_exit();
		return EXIT_FAILURE;
	}
	



	// Configure SDR
	// printf("Configuring SDR\n");
	if (rtlsdr_set_tuner_gain_mode(dev, 1)) {
		fprintf(stderr, "ERROR: Failed to enable manual gain.\n");
		exit(1);
	}
	if (rtlsdr_set_tuner_gain(dev, gain)) {
		fprintf(stderr, "ERROR: Failed to set tuner gain.\n");
		exit(1);
	}
	if (rtlsdr_set_center_freq(dev, center_freq)) {
		fprintf(stderr, "ERROR: Failed to set center frequency.\n");
		exit(1);
	}
	if (rtlsdr_set_sample_rate(dev, samp_freq)) {
		fprintf(stderr, "ERROR: Failed to set sampling frequency.\n");
		exit(1);
	}
	if (rtlsdr_reset_buffer(dev)) {
		fprintf(stderr, "ERROR: Failed to reset buffers.\n");
		exit(1);
	}
	// printf("SDR Configured.\n");

	// Configure signal handlers
	// printf("Configuring signal handlers.\n");
	signal(SIGINT, sighandler);
	signal(SIGTERM, sighandler);
	// printf("Signal handlers configured\n");

	// Configure worker thread
	// printf("Configuring worker thread\n");
	if (pthread_attr_init(&attr)) {
		fprintf(stderr, "ERROR: Failed to create default thread attributes.\n");
		exit(1);
	}
	pargs.run_num = run_num;
	pargs.frame_len = block_size;
	if (pthread_create(&thread_id, &attr, proc_queue, (void*) &pargs)) {
		fprintf(stderr, "ERROR: Failed to create detached thread.\n");
		exit(1);
	}
	// printf("Worker thread configured\n");

	// Get timestamp
	printf("Starting record\n");
	clock_gettime(CLOCK_REALTIME, &start_time);
	// begin recording
	
 	//rtlsdr_read_async(dev, rtlsdr_callback, (void*) &data_queue, 0, block_size);
	
	result = airspy_start_rx(device, airspy_callback, NULL);
	if( result != AIRSPY_SUCCESS ) {
		printf("airspy_start_rx() failed: %s (%d)\n", airspy_error_name(result), result);
		airspy_close(device);
		airspy_exit();
		return EXIT_FAILURE;
	}

	result = airspy_set_freq(device, center_freq);
	if( result != AIRSPY_SUCCESS ) {
		printf("airspy_set_freq() failed: %s (%d)\n", airspy_error_name(result), result);
		airspy_close(device);
		airspy_exit();
		return EXIT_FAILURE;
	}


	printf("AIRSPY_Recording\n");

	// clean up
	// Add timing data
	char buf[256];
	snprintf(buf, sizeof(buf), "%s/%s%06d", DATA_DIR, META_PREFIX, run_num);
	FILE* timing_stream = fopen(buf, "w");
	fprintf(timing_stream, "start_time: %f\n",
	        start_time.tv_sec + (float)start_time.tv_nsec / 1.e9);
	fprintf(timing_stream, "center_freq: %d\n", center_freq);
	fprintf(timing_stream, "sampling_freq: %d\n", air_samp_freq);
	fprintf(timing_stream, "gain: %f\n", gain / 10.0);
	fclose(timing_stream);

	printf("Stopping record\n");
	printf("Queued %f seconds of data\n", num_samples / 2048000.0);
	pthread_join(thread_id, NULL);
	rtlsdr_close(dev);
	airspy_close(device);
}

/**
 * Signal handler for this program.  Cancels the asynchronous read for the SDR
 * and notifies all threads to finish execution.
 */
void sighandler(int signal) {
	printf("Signal caught, exiting\n");
	run = 0;
	printf("Closing airspy and rtlsdr\n");
	rtlsdr_cancel_async(dev);
	airspy_stop_rx(device);
}

/**
 * Worker thread function.  Reads information from the queue and writes it to
 * the disk.
 */
void* proc_queue(void* args) {
	struct proc_queue_args* pargs = (struct proc_queue_args*) args;
	int run_num = pargs->run_num;
	int frame_len = pargs->frame_len;
	FILE* data_stream = NULL;
	char buff[256];
	int frame_num;
	uint64_t num_samples = 0;
	int file_num = 0;

	uint8_t data_buf[frame_len];

	frame_num = 0;
	// Lock mutex
	bool empty = true;
	pthread_mutex_lock(&lock);
	empty = queue_isEmpty(&data_queue);
	pthread_mutex_unlock(&lock);
	sleep(1);


	while (run || !empty) {
		
		int result;
		printf("RUN: %d\tLENGTH: %d\n", run, data_queue.length);
		lock_mutex();
		empty = queue_isEmpty(&data_queue);
		
		pthread_mutex_unlock(&lock);

				
		result = airspy_is_streaming(device);
		if(result != AIRSPY_TRUE){
			

			run =  0;

		} 
		//printf(airspy_error_name(result), result,"\n");

		

		if (!empty) {
			// Process queue
			printf("Process Queue\n");
			if (frame_num / FRAMES_PER_FILE + 1 != file_num) {
				if (data_stream) {
					fclose(data_stream);
				}
				snprintf(buff, sizeof(buff),
				         "%s/RAW_DATA_%06d_%06d", DATA_DIR, run_num,
				         frame_num / FRAMES_PER_FILE + 1);
				printf("File: %s\n", buff);
				file_num++;
				data_stream = fopen(buff, "wb");
			}
			lock_mutex();
			unsigned char* data_ptr = NULL;
			data_ptr = (unsigned char*) queue_pop(&data_queue);
			pthread_mutex_unlock(&lock);

			for (int i = 0; i < frame_len; i++) {
				data_buf[i] = (uint8_t)data_ptr[i];
			}
			fwrite(data_buf, sizeof(uint8_t), frame_len, data_stream);

			free(data_ptr);
			frame_num++;
			num_samples += frame_len / 2;
		} else {
			usleep(FILE_CAPTURE_DAEMON_SLEEP_PERIOD_MS * 1000);
		}
	}
	if(data_stream){
		fclose(data_stream);
	}
	
	printf("Recorded %f seconds of data to disk\n", num_samples / 2500000.0);
	printf("Queue length at end: %d.\n", data_queue.length);
	return NULL;
}

// static void rtlsdr_callback(unsigned char* buf, uint32_t len, void *ctx) {
// 	counter++;
// 	if (counter > 1000) {
// 	}
// 	if (!run) {
// 		return;
// 	}
// 	num_samples += len/2 ;
// 	unsigned char* newframe = malloc(len * sizeof(char));
// 	for (int i = 0; i < len; i++) {
// 		newframe[i] = buf[i];
// 	}
// 	pthread_mutex_lock(&lock);
// 	queue_push((queue*)ctx, (void*) newframe);
// 	pthread_mutex_unlock(&lock);
// }

int airspy_callback(airspy_transfer_t* transfer){

	//void* airctx=malloc(sizeof(airctx));
	//void* airctx;
	 uint32_t  bytes_to_write;
	unsigned char*  pt_rx_buffer;

	
	

	counter++;
	if (counter > 1000) {
	}
	if (!run) {
		return(1);
		printf("return\n");
	}
	

	bytes_to_write = transfer->sample_count * INT16_EL_SIZE_BYTE * 2;
	pt_rx_buffer = (unsigned char *) transfer->samples;


	//num_samples += transfer->sample_count * INT16_EL_SIZE_BYTE * 2;
	unsigned char* frame = malloc(bytes_to_write * sizeof(char));
	for (int i = 0; i < bytes_to_write; i++) {
		frame[i] = pt_rx_buffer[i];

	}
	pthread_mutex_lock(&lock);
	printf("Pushing\n");
	queue_push(&data_queue, (void*) frame);
	pthread_mutex_unlock(&lock);		
	

	return(0);

}



void lock_mutex() {
	pthread_mutex_lock(&lock);
}
