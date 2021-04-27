#ifndef ALGEBRAICA_LINEAR_H
#define ALGEBRAICA_LINEAR_H

#include <cmath>
#include <algorithm>
#include <ostream>

/*
 +–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––+
 | -------------------------------------- linear operations -------------------------------------- |
 +–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––+
*/
namespace Algebraica {
  template<typename T> class linear
  {
  public:
    /*
     * Returns min(max(x, minVal), maxVal) for each component in x
     * using the floating-point values minVal and maxVal.
     * Translates this matrix by the given vector
     *
     * @param {T} x : value to clamp
     * @param {T} min_value : minimum value
     * @param {T} max_value : maximum value
     */
    const T clamp(const T &value, const T &min_value, const T &max_value);
  };

  typedef linear<double> linearD;
  typedef linear<float> linearF;
  typedef linear<int> linearI;
}

#endif // ALGEBRAICA_LINEAR_H
