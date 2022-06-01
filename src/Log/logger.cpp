#include "logger.h"

/**
 * \fun
 * This is the default constructor for the float_node struct.
 */
float_node::float_node(double v){
	value = v;
};

/**
 * \fun
 * This method is used to add a node to the list of double values.
 * @param v: double. It is the double value to add to the list.
 */
void float_list::add_node(double v){
	if( head == NULL){
		head = new float_node(v);
		tail = head;
	}else{
		tail->next = new float_node(v);
		tail = tail->next;
	};
	size++;
};

/**
 * \fun
 * This is the custom destructor for the float_list struct.
 * It is used to deallocate all the memory used by the float values.
 */
float_list::~float_list(){
	if (head != NULL){
		float_node* tmp = head;
		while(tmp->next != NULL){
			float_node* prev = tmp;
			tmp = tmp->next;
			delete(prev);
		};
		head = NULL;
		tail = NULL;
		size = 0;
	};
};

/**
 * \fun
 * This method is the default constructor for the logger class.
 * @param path_to_logger: string. It is the path in which the logging file
 * will be saved.
 */
logger::logger(string path_to_logger){
	set_log_path(path_to_logger);
};

/**
 * \fun
 * This method customizes the destructor of the logger class.
 */
logger::~logger(){
	events->~float_list();;
};

/**
 * \fun
 * This method is used to set the location in which the logger file will be
 * written/updated.
 * @param path: string. It is the path to the file.
 */
void logger::set_log_path(string path){
	default_log_file_path = path;
	
	// Initialize the file in the new path
	FILE* log = fopen(get_log_path(), "a");
	fprintf(log, "DATE        - TIME     - ELAPSED TIME\n");
	fclose(log);
};

/**
 * \fun
 * This method is used to return an array of chars containing the location
 * of the logging file.
 */
const char* logger::get_log_path(){
	return default_log_file_path.c_str();
};

/**
 * \fun
 * This method is used to add a new event to the logging file.
 * @param event: string. It is the message describing the event to be added.
 */
void logger::add_event(string event){
	FILE* log = fopen(get_log_path(), "a");

	time_t timer = time(0);
	tm* event_time = localtime(&timer);
	
	float t = float(clock())/CLOCKS_PER_SEC;
	double clocks = 0;
	if ( events == NULL ){
		events = new float_list();
	}else{
		clocks = t - events->tail->value;
	};
	events->add_node(t);

	fprintf(log, "[%02d/%02d/%04d - %02d:%02d:%02d - %0.6f]: %s\n",
		   event_time->tm_mday, event_time->tm_mon, 1900 + event_time->tm_year,
		   event_time->tm_hour, event_time->tm_min, event_time->tm_sec,
		   clocks,
		   event.c_str());

	fclose(log);
};

/**
 * \fun
 * This method is used to wipe out the data contained in the logging file.
 * The file itself will not be deleted from the disk.
 */
void logger::clear(){
	FILE* log = fopen(get_log_path(), "w");
	fclose(log);
	events->~float_list();
	events->size = 0;
};

/**
 * \fun
 * This method is used to print the content of the logging file.
 * The method allows also for an implicit update of the logger structure
 * from a saved file. If the flag \"update\" will be set to true all the
 * lines read from the file will be added as events inside the logger
 * structure.
 * @param update: bool. It is the flag telling whether or not to add
 * the read events to the logger structure.
 */
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
		events->size = recorded_events; 
	};

};


/**
 * \fun
 * This method prints the path of the logging file and the number of events
 * recorded.
 */
void logger::info(){
	cout << "Default path is: " << default_log_file_path << endl <<
			"Events recorded: " << events->size << endl;
};
