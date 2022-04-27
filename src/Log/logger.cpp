#include "logger.h"

void logger::set_log_path(string path){
	default_log_file_path = path;
};

const char* logger::get_log_path(){
	return default_log_file_path.c_str();
};

void logger::add_event(string event){
	FILE* log = fopen(get_log_path(), "a");

	time_t timer = time(0);
	tm* event_time = localtime(&timer);

	fprintf(log, "[%02d/%02d/%04d - %02d:%02d:%02d]: %s\n",
		   event_time->tm_mday, event_time->tm_mon, 1900 + event_time->tm_year,
		   event_time->tm_hour, event_time->tm_min, event_time->tm_sec,
		   event.c_str());

	fclose(log);

	n_events += 1;
};

void logger::clear(){
	FILE* log = fopen(get_log_path(), "w");
	fclose(log);
	n_events = 0;
};

void logger::history(bool update){
	FILE* log = fopen(get_log_path(), "r");
	if (log == NULL){
		printf("No log file found\n");
		return;
	};

	int recorded_events = 0;
	char event[1000];

	while(fgets(event, 1000, log)){
		recorded_events += 1;
		printf("%d: %s\n", recorded_events, event);
	};

	fclose(log);

	if(update){
		n_events = recorded_events; 
	};

};

void logger::info(){
	cout << "Default path is: " << default_log_file_path << endl <<
			"Events recorded: " << n_events << endl;
};
