  
  // Make a scan of a circle
//   for(unsigned int i = 0; i < cloud->width; ++i)
//   {
//     for(unsigned int j = 0; j < cloud->width; ++j)
//     {
//       if(sqrt(i*i + j*j) < 100) // the point is in a circle
//       {
//         PointType p;
//         p.x = i; p.y = j; p.z = 10;
//         (*cloud)(i,j) = p;
//       }
//       else
//       {
//         PointType p;
//         p.x = std::numeric_limits<float>::quiet_NaN(); p.y = std::numeric_limits<float>::quiet_NaN(); p.z = std::numeric_limits<float>::quiet_NaN();
//         (*cloud)(i,j) = p;
//       }
//     }
//   }
