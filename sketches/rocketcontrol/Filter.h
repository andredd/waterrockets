class FloatBuffer {
	float* buffer;
	int pos;
	int size;
public:
	FloatBuffer(int);
	~FloatBuffer();
	int addVal(float);
	int getSize();
	float getVal(int);
	float getMean();
	void reset();
};

FloatBuffer::FloatBuffer(int size){
	buffer = new float[size];
	this->size = size;
	reset();
}

FloatBuffer::~FloatBuffer(){
	delete[] buffer;
}

int FloatBuffer::addVal(float val){
	int wrPos = pos;
	this->buffer[pos] = val;
	pos++;
	if( pos >= size)
		pos = 0;
	return wrPos;
}

float FloatBuffer::getVal(int pos){
	return this->buffer[pos];
}

float FloatBuffer::getMean(){
	double sum = 0;
	for( int i=0; i<size; i++)
	{
		sum += buffer[i];
	}
	return (float)(sum / size);
}

int FloatBuffer::getSize(){
	return size;
}

void FloatBuffer::reset(){
	for( int i=0; i<size; i++)
	{
		buffer[i] = 0;
	}
	pos = 0;
}