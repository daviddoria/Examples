accumulator_set< int, features< tag::min, tag::max > > acc;
acc( 2 );
acc( -1 );
acc( 1 );

// This displays "(-1, 2)"
std::cout << '(' << min( acc ) << ", " << max( acc ) << ")\n";