
enum log_level_type {LEVEL_NONE = 0, LEVEL_ERROR = 1, LEVEL_INFO = 2, LEVEL_DEBUG = 3};

void logger_set_level(int8_t level_to_set);

void log_error(char * string_buffer);

void log_info(char * string_buffer);

void log_debug(char * string_buffer);
