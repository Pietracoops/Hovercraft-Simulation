
// The following routines are from the book "Numerical Recipes in C" 
// -- this book has many other useful numerical functions along with
// c++ code for them in files.

// simple random number generator that makes one random number sequence
// with no seed value required
double ran();

// recommended routine
// call with any negative initial integer value id to initialize the sequence
// thereafter do not modify id
double ran1(long &id);

// improvement over ran2 but slower, recommended for long periods
// call with any negative initial integer value id to initialize the sequence
// thereafter do not modify id
double ran2(long &id);

// fast alternative approach useful for comparison to other routines
// call with any negative initial integer value id to initialize the sequence
// thereafter do not modify id
double ran3(long &id);

