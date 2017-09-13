template <typename T>
class SimpleBuffer {
	T* buffer;
	int pos;
	int size;
	double mean;
public:
  SimpleBuffer(int size){
	  buffer = new T[size];
	  this->size = size;
	  reset();
  }
  
  ~SimpleBuffer(){
	  delete[] buffer;
  }

  int addVal(T val){
  	int wrPos = pos;
  	this->buffer[pos] = val;
  	pos++;
  	if( pos >= size)
  		pos = 0;
  	mean = (mean * (size-1) + val) / size;
  	return wrPos;
  }

  T getVal(int pos){
  	return this->buffer[pos];
  }

  double getMean(){
  	return mean;
  }

  int getSize(){
  	return size;
  }

  void reset(){
  	for( int i=0; i<size; i++)
  	{
  		buffer[i] = 0;
  	}
  	pos = 0;
  	mean = 0;
  }
};
