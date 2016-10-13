#include <cstdio>
#include <string.h>
#include "pfmLoader.h"


 



void HDRImage::setSize(int x, int y)
{
 if(mSizeX==x && mSizeY==y)
 return;
 mSizeX = x;
 mSizeY = y;
 //if (data)
 //delete[] data;
 data = new COLOR[mSizeX* mSizeY];
}
 
void HDRImage::loadPfm(const char * filename)
{
 
    char strPF[3];
    unsigned int SizeX;
    unsigned int SizeY;
    float dummy;
    int dummyC;
 
    FILE * file = fopen(filename, "rb");
 
    if (file == NULL) {
        printf("PFM-File not found!\n");
        return;
    }
 
    fscanf(file, "%s\n%u %u\n", strPF, &SizeX, &SizeY);
    dummyC = fgetc(file);
    fscanf(file, "\n%f\n", &dummy);
 
    //DEBUG Ausgabe
    printf("Keyword: %s\n",strPF);
    printf("Size X: %d\n", SizeX);
    printf("Size Y: %d\n", SizeY);
    printf("dummy: %f\n", dummy);
    // ENDE Debug Ausgabe
 
    this->setSize(SizeX, SizeY);
 
    int result;
    int lSize;
    lSize = mSizeX*3;
    for(int y=mSizeY-1; y>=0; y--)
    {
        result = fread(data+1+mSizeX*y, sizeof(float), lSize, file);
        if (result != lSize) {
            printf("Error reading PFM-File. %d Bytes read.\n", result);
        }
    }
 
    fclose(file);
}
 
void HDRImage::writePfm(const char * filename)
{
 
 char sizes[256];
 FILE * file = fopen(filename, "wb");
 
 fwrite("PF\n",sizeof(char), 3, file);
 sprintf(sizes, "%d %d\n", mSizeX, mSizeY);
 
 fwrite(sizes,sizeof(char), strlen(sizes)+1, file);
 fwrite("\n",sizeof(char),1, file);
 fwrite("-1.000000\n",sizeof(char),10, file);
 
 for(int y=mSizeY-1; y>=0; y--)
 fwrite(data+1+mSizeX*y, sizeof(float), mSizeX*3, file);
 
 fclose(file);
}