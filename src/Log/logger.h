#ifndef __LOGGER_H__
#define __LOGGER_H__

#include <time.h>
#include <string>
#include <cstdio>
#include <iostream>

using namespace std;

/**
 * This structure is used as a storage for the clock time used by the program.
 * @param value: double. It is the value of type double contained
 * @param next: float_node\*. It is the pointer to the next double value.
 */
typedef struct float_node{
	double value = 0.0;
	float_node* next = NULL;

	float_node(double v);
}float_node;

/**
 * This struct is the list used to store the clock values.
 * @param head: float_node\*. It is the pointer to the head of the list;
 * @param tail: float_node\*. It is the pointer to the tail of the list;
 * @param size: int. It is the current size of the list.
 * 
 * Methods available are:
 * @see add_node(): It allows to add a new node to the list.
 * @see ~float_lost(): It is the custom destructor of the list.
 */
typedef struct float_list{
	float_node* head = NULL;
	float_node* tail = NULL;
	int size = 0;

	void add_node(double v);
	~float_list();
}float_list;

/**
 * This class is an abstraction of a log file. It allows to write what the
 * program does to a file with the knowledge of time and date in which events
 * occurs.
 * Parameters:
 * @param default_log_file_path: string. Is the path in which the log.txt file
 * is going to be saved and updated.
 * @param n_events: int. Is the number of events that have been recorded.
 * The methods available are:
 * @see logger(): It is the zero parameters constructor of the logger class.
 * @see get_log_path(): returns the path in which the log.txt is saved.
 * @see set_log_path(string path): change the path in which the log.txt is.
 * @see add_event(string event): adds a new record to the log.
 * @see info(): prints a summary of the records contained in the log.
 * @see history(bool update): prints all the records in the log file. If update
 * is set to true it will update the n_events private parameter.
 * @see clear(): is the function responsible to wipe out the log file.
 * @see ~logger(): It is the custom destructor of the logger class.
 */
typedef class logger{
	private:
		string default_log_file_path;
		float_list *events = NULL;
	public:
		logger(string path_to_logger="log.txt");
		const char* get_log_path();
		void set_log_path(string path);
		void add_event(string event);
		void info();
		void history(bool update=false);
		void clear();
		~logger();
}logger;

#endif
