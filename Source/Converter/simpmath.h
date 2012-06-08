extern double fint( double );
extern unsigned int GetExt( char* fn );
extern void SetExt( char** fn, const char* ext );
extern void reversecode4( unsigned int* code );
static inline char tolower(char c) { return ( c >= 'A' && c <= 'Z' ) ? c + 'a' - 'A' : c; }
