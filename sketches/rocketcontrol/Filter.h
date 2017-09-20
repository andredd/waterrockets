template <typename T>
class SimpleBuffer {
  protected:
    T* buffer;
    int pos;
    int size;
    bool prealloc = false;
    long total = 0;
  public:

    SimpleBuffer(int size) {
      buffer = new T[size];
      this->size = size;
      reset();
    }

    SimpleBuffer(T* buf, int size) {
      buffer = buf;
      this->size = size;
      prealloc = true;
      reset();
    }


    ~SimpleBuffer() {
      if( !prealloc) delete[] buffer;
    }

    int addVal(T val) {
      int wrPos = pos;
      this->buffer[pos] = val;
      pos++;
      total++;
      if ( pos >= size)
        pos = 0;

      return wrPos;

    }

    int getPos(){
      return pos;
    }
    
    T getVal(int pos) {
      return this->buffer[pos];
    }

    int getSize() {
      return size;
    }

    int getTotal(){
      return total;
     }
     
    void reset() {
      memset(buffer, 0, sizeof(buffer));
      pos = 0;
      total = 0;
    }
};

template <typename R>
class RollMeanBuffer: public SimpleBuffer<R>
{
  private:
    double mean;

  public:
    RollMeanBuffer(int size) : SimpleBuffer<R>(size){
      mean = 0;
    }

    double getMean() {
      return mean;
    }

    void reset() {
      SimpleBuffer<R>::reset();
      mean = 0;
    }

    int addVal(R val)
    {
      int res = SimpleBuffer<R>::addVal(val);
      mean = (mean * (this->size - 1) + val) / this->size;
      return res;
    }
};
