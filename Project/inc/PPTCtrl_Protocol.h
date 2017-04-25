#ifndef __PPT_CTRL_PROTOCOL_H
#define __PPT_CTRL_PROTOCOL_H

// Object
#define PPT_OBJ_PC             		'P'     // PC
#define PPT_OBJ_APP              	'A'     // Application
#define PPT_OBJ_SERVICE            	'S'     // Backgroud Service
#define PPT_OBJ_SCREEN                  's'     // Screen
#define PPT_OBJ_MEDIA              	'm'     // Media
#define PPT_OBJ_PAGE              	'p'     // Page

// Movement
#define CONTENT_GO_FIRST                'f'     // Goto first
#define CONTENT_GO_LAST            	'l'     // Goto last
#define CONTENT_GO_NEXT                 'n'     // Goto next
#define CONTENT_GO_PREV                 'p'     // Goto previous
#define CONTENT_REPLAY       		'R'     // Replay from beginning
#define CONTENT_FAST_FORWARD       	'F'     // Fast Forward
#define CONTENT_FAST_BACKWARD     	'B'     // Fast Backward

#endif /* __PPT_CTRL_PROTOCOL_H */