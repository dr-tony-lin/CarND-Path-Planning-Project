#ifndef _UTILS_H_
#define _UTILS_H_
#include <vector>
#include <deque>
#include <math.h>
#include "Eigen/Eigen"

#ifndef M_PI
    #define M_PI       3.14159265358979323846   // pi
    #define M_PI_2     1.57079632679489661923   // pi/2
    #define M_PI_4     0.785398163397448309616  // pi/4
#endif

/**
 * Mile per hour to meters per second
 * @param mph the value to convert
 */ 
inline double MpH2MpS(double mph) {
  return mph * 1609.34 / 3600.0;
}

/**
 * Meter per second to mile per hour
 * @param mps the value to convert
 */ 
inline double MpS2MpH(double mps) {
  return mps * 3600.0 / 1609.34;
}

/**
 * Evaluate the given polynomial
 * @param coeffs the coefficients
 * @param x the variable
 */ 
extern double polyeval(Eigen::VectorXd coeffs, const double x);

/**
 * Compute the derivative of the given polynomial.
 * @param coeffs the coefficients
 * @param x the variable
 */ 
extern double polyder(const Eigen::VectorXd coeffs, const double &x);

/**
 * Clamp a to min and max range
 * @param a the value to clamp
 * @param min the minimal value
 * @param max the maximal value
 * @retur the clampped value
 */
template<typename T> T clamp(const T a, const T min, const T max) {
  return a < min? min: (a > max? max: a);
}

// Fit a polynomial to the given set of points
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
extern Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

inline double sigmoid(const double x) {return 1.0 / (1 + exp(-x));}

/**
 * Convert degree to radian
 * @param x degree to convert
 */ 
inline double deg2rad(double x) { return x * M_PI / 180; }

/**
 * Convert radian to degree
 * @param x radian to convert
 */
inline double rad2deg(double x) { return x * 180 / M_PI; }

/**
 * Compute square, as not sure if pow if optimized
 * @param a the value to square
 */ 
template<typename T> T square(const T &a) {return a * a;}

/**
 * Normalize the given angle to [-PI, PI) range
 * @param a the angle
 */                      
template<typename T> T normalizeAngle(const T &a) {
  T result = a;
  while (result >= M_PI) result -= 2. * M_PI;
  while (result < -M_PI) result += 2. * M_PI;
  return result;
}

inline double dot(double x1, double y1, double x2, double y2) {
  return x1 * x2 + y1 * y2;
}

inline double cross(double x1, double y1, double x2, double y2) {
  return x1 * y2 - x2 * y1;
}

inline double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

inline double length(double x, double y) {
  return sqrt(x * x + y * y);
}

std::vector<double> unitv(double x, double y);

/// Calculate the Jerk Minimizing Trajectory that connects the initial state
/// to the final state in time t.
std::vector<double> JMT(std::vector< double> start, std::vector<double> end, double t);

/// Fixed length buffer using deque
template <
    class T,
    class Allocator = std::allocator<T>
> class FixedBuffer: protected std::deque<T, Allocator> {
private:
  size_t maxSize;

public:
  using reference = T&;
  using const_reference = const T&;
  using iterator = typename std::deque<T, Allocator>::iterator;
  using const_iterator = typename std::deque<T, Allocator>::const_iterator;

  FixedBuffer(const size_t size) {
    this->maxSize = size;
  };

  size_t size() const {
    return std::deque<T, T>::size();
  }

  size_t capacity() {
    return maxSize;
  }

  FixedBuffer& operator=(FixedBuffer &another) {
    return std::deque<T, Allocator>::operator=(another);
  }

  reference operator[] (size_t n) { return std::deque<T, Allocator>::operator[](n);};
  const_reference operator[] (size_t n) const { return std::deque<T, Allocator>::operator[](n);};
  iterator begin() { return std::deque<T, Allocator>::begin();};
  const_iterator begin() const { return std::deque<T, Allocator>::begin();};
  iterator end() { return std::deque<T, Allocator>::end();};
  const_iterator end() const { return std::deque<T, Allocator>::end();};

  void push(const T& val) {
    this->push_back(val);
    if (this->size() > maxSize) {
      this->pop_front();
    }
  };

  T& pop() {
    if (this->size() > 0) {
      reference ret = this->front(); 
      this->pop_front();
      return ret;
    }
    throw "Illegal operation - pop from empty collection!";
  }

  template <class... Args> void emplace(Args&&... args) {
    this->emplace_back(args...);
    if (this->size() > maxSize) {
      this->pop_front();
    }
  };
};

// Small number
const double EPSILON = 1E-8;

#endif
