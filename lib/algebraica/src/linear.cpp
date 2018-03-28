
#include "algebraica/linear.h"

namespace Algebraica {  
  template<typename T>
  const T linear<T>::clamp(const T &value, const T &min_value, const T &max_value){
    return std::min(std::max(value, min_value), max_value);
  }

  template class linear<double>;
  template class linear<float>;
  template class linear<int>;
}
