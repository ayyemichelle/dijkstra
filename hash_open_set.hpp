#ifndef HASH_OPEN_SET_HPP_
#define HASH_OPEN_SET_HPP_

#include <string>
#include <iostream>
#include <sstream>
#include <initializer_list>
#include "ics_exceptions.hpp"
#include "pair.hpp"


namespace ics {


#ifndef undefinedhashdefined
#define undefinedhashdefined
template<class T>
int undefinedhash (const T& a) {return 0;}
#endif /* undefinedhashdefined */

//Instantiate the templated class supplying thash(a): produces a hash value for a.
//If thash is defaulted to undefinedhash in the template, then a constructor must supply chash.
//If both thash and chash are supplied, then they must be the same (by ==) function.
//If neither is supplied, or both are supplied but different, TemplateFunctionError is raised.
//The (unique) non-undefinedhash value supplied by thash/chash is stored in the instance variable hash.
template<class T, int (*thash)(const T& a) = undefinedhash<T>> class HashOpenSet {
  public:
    typedef int (*hashfunc) (const T& a);

    //Destructor/Constructors
    ~HashOpenSet ();

    HashOpenSet (double the_load_threshold = 1.0, int (*chash)(const T& a) = undefinedhash<T>);
    explicit HashOpenSet (int initial_bins, double the_load_threshold = 1.0, int (*chash)(const T& k) = undefinedhash<T>);
    HashOpenSet (const HashOpenSet<T,thash>& to_copy, double the_load_threshold = 1.0, int (*chash)(const T& a) = undefinedhash<T>);
    explicit HashOpenSet (const std::initializer_list<T>& il, double the_load_threshold = 1.0, int (*chash)(const T& a) = undefinedhash<T>);

    //Iterable class must support "for-each" loop: .begin()/.end() and prefix ++ on returned result
    template <class Iterable>
    explicit HashOpenSet (const Iterable& i, double the_load_threshold = 1.0, int (*chash)(const T& a) = undefinedhash<T>);


    //Queries
    bool empty      () const;
    int  size       () const;
    bool contains   (const T& element) const;
    std::string str () const; //supplies useful debugging information; contrast to operator <<

    //Iterable class must support "for-each" loop: .begin()/.end() and prefix ++ on returned result
    template <class Iterable>
    bool contains_all (const Iterable& i) const;


    //Commands
    int  insert (const T& element);
    int  erase  (const T& element);
    void clear  ();

    //Iterable class must support "for" loop: .begin()/.end() and prefix ++ on returned result

    template <class Iterable>
    int insert_all(const Iterable& i);

    template <class Iterable>
    int erase_all(const Iterable& i);

    template<class Iterable>
    int retain_all(const Iterable& i);


    //Operators
    HashOpenSet<T,thash>& operator = (const HashOpenSet<T,thash>& rhs);
    bool operator == (const HashOpenSet<T,thash>& rhs) const;
    bool operator != (const HashOpenSet<T,thash>& rhs) const;
    bool operator <= (const HashOpenSet<T,thash>& rhs) const;
    bool operator <  (const HashOpenSet<T,thash>& rhs) const;
    bool operator >= (const HashOpenSet<T,thash>& rhs) const;
    bool operator >  (const HashOpenSet<T,thash>& rhs) const;

    template<class T2, int (*hash2)(const T2& a)>
    friend std::ostream& operator << (std::ostream& outs, const HashOpenSet<T2,hash2>& s);



  public:
    class Iterator {
      public:
        //Private constructor called in begin/end, which are friends of HashOpenSet<T,thash>
        ~Iterator();
        T           erase();
        std::string str  () const;
        HashOpenSet<T,thash>::Iterator& operator ++ ();
        HashOpenSet<T,thash>::Iterator  operator ++ (int);
        bool operator == (const HashOpenSet<T,thash>::Iterator& rhs) const;
        bool operator != (const HashOpenSet<T,thash>::Iterator& rhs) const;
        T& operator *  () const;
        T* operator -> () const;
        friend std::ostream& operator << (std::ostream& outs, const HashOpenSet<T,thash>::Iterator& i) {
          outs << i.str(); //Use the same meaning as the debugging .str() method
          return outs;
        }
        friend Iterator HashOpenSet<T,thash>::begin () const;
        friend Iterator HashOpenSet<T,thash>::end   () const;

      private:
        //If can_erase is false, current indexes the "next" value (must ++ to reach it)
        int                 current; //Bin Index
        HashOpenSet<T,thash>*   ref_set;
        int                 expected_mod_count;
        bool                can_erase = true;

        //Called in friends begin/end
        Iterator(HashOpenSet<T,thash>* iterate_over, int initial);
    };


    Iterator begin () const;
    Iterator end   () const;


private:
  int (*hash)(const T& k);      //Hashing function used (from template or constructor)
  enum bin_state {bs_empty, bs_occupied, bs_was_occupied};
  T* set           = nullptr;    //T array
  bin_state* state = nullptr;    //bin_state[] describes the state of set[i]
  double load_threshold;         //used/bins <= load_threshold
  int bins         = 1;          //# bins in array (should start at 1 so hash_compress doesn't % 0)
  int used         = 0;          //Cache for number of elements in the hash table
  int mod_count    = 0;          //For sensing concurrent modification


  //Helper methods
  int   hash_compress      (const T& element)  const;  //hash function ranged to [0,bins-1]
  int   find_element       (const T& kelement) const;  //Returns index to element's bin or -1
  int   next_unoccupied    (int bin)           const;  //this bin or next beyond that is unoccupied

  void  ensure_load_threshold(int new_used);           //Reallocate if load_factor > load_threshold
};





//HashOpenSet class and related definitions

////////////////////////////////////////////////////////////////////////////////
//
//Destructor/Constructors

template<class T, int (*thash)(const T& a)>
HashOpenSet<T,thash>::~HashOpenSet() {
  delete [] set;
  delete [] state;
}


template<class T, int (*thash)(const T& a)>
HashOpenSet<T,thash>::HashOpenSet(double the_load_threshold, int (*chash)(const T& element))
: hash(thash != (hashfunc)undefinedhash<T> ? thash : chash), load_threshold(the_load_threshold) {
  if (hash == (hashfunc)undefinedhash<T>)
    throw TemplateFunctionError("HashOpenSet::default constructor: neither specified");
  if (thash != (hashfunc)undefinedhash<T> && chash != (hashfunc)undefinedhash<T> && thash != chash)
    throw TemplateFunctionError("HashOpenSet::default constructor: both specified and different");
  if (the_load_threshold > 1.) {
    std::ostringstream answer;
    answer << "HashOpenSet::default constructor: the_load_threshold(" << the_load_threshold << ") > 1";
    throw IcsError(answer.str());
  }

  set   = new T        [bins];
  state = new bin_state[bins];
  for (int b=0; b<bins; ++b)
    state[b] = bs_empty;
}


template<class T, int (*thash)(const T& a)>
HashOpenSet<T,thash>::HashOpenSet(int initial_bins, double the_load_threshold, int (*chash)(const T& element))
: hash(thash != (hashfunc)undefinedhash<T> ? thash : chash), bins(initial_bins), load_threshold(the_load_threshold) {
  if (hash == (hashfunc)undefinedhash<T>)
    throw TemplateFunctionError("HashOpenSet::length constructor: neither specified");
  if (thash != (hashfunc)undefinedhash<T> && chash != (hashfunc)undefinedhash<T> && thash != chash)
    throw TemplateFunctionError("HashOpenSet::length constructor: both specified and different");
  if (the_load_threshold > 1.) {
    std::ostringstream answer;
    answer << "HashOpenSet::length constructor: the_load_threshold(" << the_load_threshold << ") > 1";
    throw IcsError(answer.str());
  }

  if (bins < 1)
    bins = 1;
  set   = new T        [bins];
  state = new bin_state[bins];
  for (int b=0; b<bins; ++b)
    state[b] = bs_empty;
}


template<class T, int (*thash)(const T& a)>
HashOpenSet<T,thash>::HashOpenSet(const HashOpenSet<T,thash>& to_copy, double the_load_threshold, int (*chash)(const T& element))
: hash(thash != (hashfunc)undefinedhash<T> ? thash : chash), load_threshold(the_load_threshold), bins(to_copy.bins) {
  if (hash == (hashfunc)undefinedhash<T>)
    hash = to_copy.hash;//throw TemplateFunctionError("HashOpenSet::copy constructor: neither specified");
  if (thash != (hashfunc)undefinedhash<T> && chash != (hashfunc)undefinedhash<T> && thash != chash)
    throw TemplateFunctionError("HashOpenSet::copy constructor: both specified and different");
  if (the_load_threshold > 1.) {
    std::ostringstream answer;
    answer << "HashOpenSet::length constructor: the_load_threshold(" << the_load_threshold << ") > 1";
    throw IcsError(answer.str());
  }

  if (hash == to_copy.hash && (double)to_copy.size()/to_copy.bins <= the_load_threshold) {
    used = to_copy.used;
    set   = new T        [bins];
    state = new bin_state[bins];
    for (int b = 0; b < bins; ++b) {
      state[b] = to_copy.state[b];
      if (state[b] == bs_occupied)
        set [b] = to_copy.set[b];
    }
  }else {
    bins = std::max(1,int(to_copy.size()/load_threshold));
    set   = new T        [bins];
    state = new bin_state[bins];
    for (int b=0; b<bins; ++b)
      state[b] = bs_empty;

    for (int b=0; b<to_copy.bins; ++b)
      if (to_copy.state[b] == bs_occupied)
        insert(to_copy.set[b]);
  }
}


template<class T, int (*thash)(const T& a)>
HashOpenSet<T,thash>::HashOpenSet(const std::initializer_list<T>& il, double the_load_threshold, int (*chash)(const T& element))
: hash(thash != (hashfunc)undefinedhash<T> ? thash : chash), load_threshold(the_load_threshold), bins(std::max(1,int(il.size()/the_load_threshold))) {
  if (hash == (hashfunc)undefinedhash<T>)
    throw TemplateFunctionError("HashOpenSet::initializer_list constructor: neither specified");
  if (thash != (hashfunc)undefinedhash<T> && chash != (hashfunc)undefinedhash<T> && thash != chash)
    throw TemplateFunctionError("HashOpenSet::initializer_list constructor: both specified and different");
  if (the_load_threshold > 1.) {
    std::ostringstream answer;
    answer << "HashOpenSet::initializer_list constructor: the_load_threshold(" << the_load_threshold << ") > 1";
    throw IcsError(answer.str());
  }

  set   = new T        [bins];
  state = new bin_state[bins];
  for (int b=0; b<bins; ++b)
    state[b] = bs_empty;

  for (const T& s_element : il)
    insert(s_element);
}


template<class T, int (*thash)(const T& a)>
template<class Iterable>
HashOpenSet<T,thash>::HashOpenSet(const Iterable& i, double the_load_threshold, int (*chash)(const T& a))
: hash(thash != (hashfunc)undefinedhash<T> ? thash : chash), load_threshold(the_load_threshold), bins(std::max(1,int(i.size()/the_load_threshold))) {
  if (hash == (hashfunc)undefinedhash<T>)
    throw TemplateFunctionError("HashOpenSet::Iterable constructor: neither specified");
  if (thash != (hashfunc)undefinedhash<T> && chash != (hashfunc)undefinedhash<T> && thash != chash)
    throw TemplateFunctionError("HashOpenSet::Iterable constructor: both specified and different");
  if (the_load_threshold > 1.) {
    std::ostringstream answer;
    answer << "HashOpenSet::Iterable constructor: the_load_threshold(" << the_load_threshold << ") > 1";
    throw IcsError(answer.str());
  }

  set   = new T        [bins];
  state = new bin_state[bins];
  for (int b=0; b<bins; ++b)
    state[b] = bs_empty;

  for (const T& s_element : i)
    insert(s_element);
}


////////////////////////////////////////////////////////////////////////////////
//
//Queries

template<class T, int (*thash)(const T& a)>
bool HashOpenSet<T,thash>::empty() const {
  return used == 0;
}


template<class T, int (*thash)(const T& a)>
int HashOpenSet<T,thash>::size() const {
  return used;
}


template<class T, int (*thash)(const T& a)>
bool HashOpenSet<T,thash>::contains (const T& element) const {
  return find_element(element) != -1;
}


template<class T, int (*thash)(const T& a)>
std::string HashOpenSet<T,thash>::str() const {
  std::ostringstream answer;
  answer << "HashOpenSet[";
  if (bins != 0) {
    answer << std::endl;
    for (int b=0; b<bins; ++b) {
      answer << "  bin[" << b << "] = ";
      switch (state[b]) {
        case bs_empty        : answer << "empty"                 << std::endl; break;
        case bs_occupied     : answer << "occupied : " << set[b] << std::endl; break;
        case bs_was_occupied : answer << "was occupied : "       << std::endl; break;
      }
    }
  }
  answer  << "](load_threshold=" << load_threshold << ",bins=" << bins << ",used=" <<used <<",mod_count=" << mod_count << ")";
  return answer.str();
}


template<class T, int (*thash)(const T& a)>
template <class Iterable>
bool HashOpenSet<T,thash>::contains_all(const Iterable& i) const {
  for (const T& v : i)
    if (!contains(v))
      return false;

  return true;
}


////////////////////////////////////////////////////////////////////////////////
//
//Commands

template<class T, int (*thash)(const T& a)>
int HashOpenSet<T,thash>::insert(const T& element) {
  int c = find_element(element);
  if (c != -1)
      return 0;

  ensure_load_threshold(used+1);

  ++used;
  ++mod_count;
  int bin = next_unoccupied(hash_compress(element));
  state[bin] = bs_occupied;
  set  [bin] = element;
  return 1;
}


template<class T, int (*thash)(const T& a)>
int HashOpenSet<T,thash>::erase(const T& element) {
  int c = find_element(element);
  if (c == -1) {
    return 0;
  }
  state[c] = bs_was_occupied;

  --used;
  ++mod_count;
  return 1;
}


template<class T, int (*thash)(const T& a)>
void HashOpenSet<T,thash>::clear() {
  for (int b=0; b<bins; ++b)
    state[b] = bs_empty;

  used = 0;
  ++mod_count;
}


template<class T, int (*thash)(const T& a)>
template<class Iterable>
int HashOpenSet<T,thash>::insert_all(const Iterable& i) {
  int count = 0;
  for (const T& v : i)
    count += insert(v);

  return count;
}


template<class T, int (*thash)(const T& a)>
template<class Iterable>
int HashOpenSet<T,thash>::erase_all(const Iterable& i) {
  int count = 0;
  for (const T& v : i)
    count += erase(v);
  return count;
}


template<class T, int (*thash)(const T& a)>
template<class Iterable>
int HashOpenSet<T,thash>::retain_all(const Iterable& i) {
  HashOpenSet<T,thash> s(i);

  int count = 0;
  for (int b=0; b<bins; ++b)
    if (state[b] == bs_occupied && !s.contains(set[b])) {
      state[b] = bs_was_occupied;
      ++count;
    }

  used -= count;
  return count;
}


////////////////////////////////////////////////////////////////////////////////
//
//Operators

template<class T, int (*thash)(const T& a)>
HashOpenSet<T,thash>& HashOpenSet<T,thash>::operator = (const HashOpenSet<T,thash>& rhs) {
  if (this == &rhs)
    return *this;

  if (hash == rhs.hash && (double)rhs.size()/rhs.bins <= load_threshold) {
    //Use a copy of the rhs data
    delete [] set;
    delete [] state;
    bins  = rhs.bins;
    used  = rhs.used;
    set   = new T        [bins];
    state = new bin_state[bins];

    for (int b=0; b<rhs.bins; ++b) {
      state[b] = rhs.state[b];
      if (state[b] == bs_occupied)
        set[b] = rhs.set[b];
    }
  }else {
    //Copy with rehashing
    delete[] set;
    delete[] state;
    bins = std::max(1,int(rhs.size()/load_threshold));
    set   = new T        [bins];
    state = new bin_state[bins];
    for (int b=0; b<bins; ++b)
      state[b] = bs_empty;

    for (int b=0; b<rhs.bins; ++b) {
      if (rhs.state[b] == bs_occupied)
        insert(rhs.set[b]);
    }
  }
  ++mod_count;
  return *this;
}


template<class T, int (*thash)(const T& a)>
bool HashOpenSet<T,thash>::operator == (const HashOpenSet<T,thash>& rhs) const {
  if (this == &rhs)
    return true;
  if (used != rhs.size())
    return false;

  for (int b=0; b<bins; ++b)
    if (state[b] == bs_occupied) {
      int rhs_c = rhs.find_element(set[b]);
      if  (rhs_c == -1)
        return false;
    }

  return true;
}


template<class T, int (*thash)(const T& a)>
bool HashOpenSet<T,thash>::operator != (const HashOpenSet<T,thash>& rhs) const {
  return !(*this == rhs);
}


template<class T, int (*thash)(const T& a)>
bool HashOpenSet<T,thash>::operator <= (const HashOpenSet<T,thash>& rhs) const {
  if (this == &rhs)
    return true;
  if (used > rhs.size())
    return false;

  for (int b=0; b<bins; ++b)
    if (state[b] == bs_occupied && !rhs.contains(set[b]))
      return false;

  return true;
}

template<class T, int (*thash)(const T& a)>
bool HashOpenSet<T,thash>::operator < (const HashOpenSet<T,thash>& rhs) const {
  if (this == &rhs)
    return false;
  if (used >= rhs.size())
    return false;

  for (int b=0; b<bins; ++b)
    if (state[b] == bs_occupied && !rhs.contains(set[b]))
      return false;

  return true;
}


template<class T, int (*thash)(const T& a)>
bool HashOpenSet<T,thash>::operator >= (const HashOpenSet<T,thash>& rhs) const {
  return rhs <= *this;
}


template<class T, int (*thash)(const T& a)>
bool HashOpenSet<T,thash>::operator > (const HashOpenSet<T,thash>& rhs) const {
  return rhs < *this;
}


template<class T, int (*thash)(const T& a)>
std::ostream& operator << (std::ostream& outs, const HashOpenSet<T,thash>& s) {
  outs  << "set[";

  int printed = 0;
  for (int b=0; b<s.bins; ++b)
    if (s.state[b] == s.bin_state::bs_occupied) // (true) for debugging purposes
      outs << (printed++ == 0? "" : ",") << s.set[b];

  outs << "]";
  return outs;
}


////////////////////////////////////////////////////////////////////////////////
//
//Iterator constructors

template<class T, int (*thash)(const T& a)>
auto HashOpenSet<T,thash>::begin () const -> HashOpenSet<T,thash>::Iterator {
  return Iterator(const_cast<HashOpenSet<T,thash>*>(this),0);
}


template<class T, int (*thash)(const T& a)>
auto HashOpenSet<T,thash>::end () const -> HashOpenSet<T,thash>::Iterator {
  return Iterator(const_cast<HashOpenSet<T,thash>*>(this),bins);
}


////////////////////////////////////////////////////////////////////////////////
//
//Private helper methods

template<class T, int (*thash)(const T& a)>
int HashOpenSet<T,thash>::hash_compress (const T& element) const {
  return abs(hash(element)) % bins;
}


template<class T, int (*thash)(const T& a)>
int HashOpenSet<T,thash>::find_element (const T& element) const {
  int bin = hash_compress(element);
  int bin_original = bin;
  do {
    if (state[bin] == bs_empty)
      return -1;
    if (state[bin] == bs_occupied && set[bin] == element)
      return bin;
    bin = ++bin%bins; //linear probing;
  } while (bin != bin_original);

  return -1;
}

template<class T, int (*thash)(const T& a)>
int HashOpenSet<T,thash>::next_unoccupied (int bin) const {
  for (/*bin*/; state[bin] == bs_occupied; bin = ++bin%bins) //linear probing
    ;

  //load_threshold <= 1 guarantees something is empty/was_occupied
  return bin;
}


template<class T, int (*thash)(const T& a)>
void HashOpenSet<T,thash>::ensure_load_threshold(int new_used) {
  if (double(new_used)/double(bins) <= load_threshold)
    return;
  int        old_bins  = bins;
  T*         old_set   = set;
  bin_state* old_state = state;

  bins  = 2*old_bins;
  used  = 0;
  set   = new T        [bins];
  state = new bin_state[bins];

  for (int b=0; b<bins; ++b)
    state[b] = bs_empty;

  for (int b=0; b<old_bins; ++b)
    if (old_state[b] == bs_occupied)
      insert(old_set[b]);

  delete [] old_set;
  delete [] old_state;
}






////////////////////////////////////////////////////////////////////////////////
//
//Iterator class definitions

template<class T, int (*thash)(const T& a)>
HashOpenSet<T,thash>::Iterator::Iterator(HashOpenSet<T,thash>* iterate_over, int initial)
: ref_set(iterate_over), expected_mod_count(ref_set->mod_count) {
  current = initial;
  while (current < ref_set->bins && ref_set->state[current] != bs_occupied)
    ++current;
}


template<class T, int (*thash)(const T& a)>
HashOpenSet<T,thash>::Iterator::~Iterator()
{}


template<class T, int (*thash)(const T& a)>
T HashOpenSet<T,thash>::Iterator::erase() {
  if (expected_mod_count != ref_set->mod_count)
    throw ConcurrentModificationError("HashOpenSet::Iterator::erase");
  if (!can_erase)
    throw CannotEraseError("HashOpenSet::Iterator::erase Iterator cursor already erased");
  if (current < 0 || current >= ref_set->bins)
    throw CannotEraseError("HashOpenSet::Iterator::erase Iterator cursor beyond data structure");

  can_erase = false;
  T to_return = ref_set->set[current];
  ref_set->state[current] = bs_was_occupied;

  --ref_set->used;
  ++ref_set->mod_count;
  expected_mod_count = ref_set->mod_count;

  return to_return;
}


template<class T, int (*thash)(const T& a)>
std::string HashOpenSet<T,thash>::Iterator::str() const {
  std::ostringstream answer;
  answer << ref_set->str() << "(current=" << current << ",expected_mod_count=" << expected_mod_count << ",can_erase=" << can_erase << ")";
  return answer.str();
}


template<class T, int (*thash)(const T& a)>
auto  HashOpenSet<T,thash>::Iterator::operator ++ () -> HashOpenSet<T,thash>::Iterator& {
  if (expected_mod_count != ref_set->mod_count)
    throw ConcurrentModificationError("HashOpenSet::Iterator::operator ++");

  if (current >= ref_set->bins)
    return *this;

  for (++current; current < ref_set->bins && ref_set->state[current] != bs_occupied; ++current)
    ;

  can_erase = true;

  return *this;
}


template<class T, int (*thash)(const T& a)>
auto  HashOpenSet<T,thash>::Iterator::operator ++ (int) -> HashOpenSet<T,thash>::Iterator {
  if (expected_mod_count != ref_set->mod_count)
    throw ConcurrentModificationError("HashOpenSet::Iterator::operator ++(int)");

  Iterator to_return(*this);

  if (current >= ref_set->bins)
    return to_return;

  for (++current; current < ref_set->bins && ref_set->state[current] != bs_occupied; ++current)
    ;

  can_erase = true;

  return to_return;
}


template<class T, int (*thash)(const T& a)>
bool HashOpenSet<T,thash>::Iterator::operator == (const HashOpenSet<T,thash>::Iterator& rhs) const {
  const Iterator* rhsASI = dynamic_cast<const Iterator*>(&rhs);
  if (rhsASI == 0)
    throw IteratorTypeError("HashOpenSet::Iterator::operator ==");
  if (expected_mod_count != ref_set->mod_count)
    throw ConcurrentModificationError("HashOpenSet::Iterator::operator ==");
  if (ref_set != rhsASI->ref_set)
    throw ComparingDifferentIteratorsError("HashOpenSet::Iterator::operator ==");

  return this->current == rhsASI->current;
}


template<class T, int (*thash)(const T& a)>
bool HashOpenSet<T,thash>::Iterator::operator != (const HashOpenSet<T,thash>::Iterator& rhs) const {
  const Iterator* rhsASI = dynamic_cast<const Iterator*>(&rhs);
  if (rhsASI == 0)
    throw IteratorTypeError("HashOpenSet::Iterator::operator !=");
  if (expected_mod_count != ref_set->mod_count)
    throw ConcurrentModificationError("HashOpenSet::Iterator::operator !=");
  if (ref_set != rhsASI->ref_set)
    throw ComparingDifferentIteratorsError("HashOpenSet::Iterator::operator !=");

  return this->current != rhsASI->current;
}

template<class T, int (*thash)(const T& a)>
T& HashOpenSet<T,thash>::Iterator::operator *() const {
  if (expected_mod_count !=
      ref_set->mod_count)
    throw ConcurrentModificationError("HashOpenSet::Iterator::operator *");
  if (!can_erase || current < 0 || current >= ref_set->bins)
    throw IteratorPositionIllegal("HashOpenSet::Iterator::operator * Iterator illegal: exhausted");

  return ref_set->set[current];
}

template<class T, int (*thash)(const T& a)>
T* HashOpenSet<T,thash>::Iterator::operator ->() const {
  if (expected_mod_count !=
      ref_set->mod_count)
    throw ConcurrentModificationError("HashOpenSet::Iterator::operator *");
  if (!can_erase || current < 0 || current >= ref_set->bins)
    throw IteratorPositionIllegal("HashOpenSet::Iterator::operator * Iterator illegal: exhausted");

  return &(ref_set->set[current]);
}

}

#endif /* HASH_OPEN_SET_HPP_ */
