	typedef struct {
	 float r;
	 float g;
	 float b;
	} COLOR;
 
	

class HDRImage{
public:
	void setSize(int, int);
	int mSizeX, mSizeY;
	void loadPfm(const char*);
	void writePfm(const char*);
	
	COLOR * data;
};