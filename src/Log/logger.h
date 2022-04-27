#ifndef __LOGGER_H__
#define __LOGGER_H__

#include <time.h>
#include <string>
#include <cstdio>
#include <iostream>

using namespace std;
/**
 * This class is an abstraction of a log file. It allows to write what the
 * program does to a file with the knowledge of time and date in which events
 * occurs.
 * Parameters:
 * @param default_log_file_path: string. Is the path in which the log.txt file
 * is going to be saved and updated.
 * @param n_events: int. Is the number of events that have been recorded.
 * The methods available are:
 * @see get_log_path(): returns the path in which the log.txt is saved.
 * @see set_log_path(string path): change the path in which the log.txt is.
 * @see add_event(string event): adds a new record to the log.
 * @see info(): prints a summary of the records contained in the log.
 * @see history(bool update): prints all the records in the log file. If update
 * is set to true it will update the n_events private parameter.
 * @see clear(): is the function responsible to wipe out the log file.
 */
typedef class logger{
	private:
		string default_log_file_path = "log.txt";
		int n_events = 0;
	public:
		const char* get_log_path();
		void set_log_path(string path);
		void add_event(string event);
		void info();
		void history(bool update=false);
		void clear();
}logger;

#endif
