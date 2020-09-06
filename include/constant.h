#ifndef BFM_CONSTANT_H
#define BFM_CONSTANT_H

#ifndef BFM_SHUT_UP
	#define BFM_DEBUG(fmt, ...) printf(fmt, ##__VA_ARGS__);
#else
	#define BFM_DEBUG(fmt, ...)
#endif 


enum model_write_mode {
	NONE_MODE      = 0L << 0, 
	PICK_LANDMARK 	   = 1L << 0,
	CAMERA_COORD   = 1L << 1,
	NO_EXPR        = 1L << 2,
	EXTRA_EXT_PARM = 1L << 3,
};


#endif // BFM_CONSTANT_H