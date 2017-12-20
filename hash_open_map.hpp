#ifndef HASH_OPEN_MAP_HPP_
#define HASH_OPEN_MAP_HPP_

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
template<class KEY,class T, int (*thash)(const KEY& a) = undefinedhash<KEY>> class HashOpenMap {
  public:
    typedef ics::pair<KEY,T>   Entry;
    typedef int (*hashfunc) (const KEY& a);

    //Destructor/Constructors
    ~HashOpenMap ();

    HashOpenMap          (double the_load_threshold = 1.0, int (*chash)(const KEY& a) = undefinedhash<KEY>);
    explicit HashOpenMap (int initial_bins, double the_load_threshold = 1.0, int (*chash)(const KEY& k) = undefinedhash<KEY>);
    HashOpenMap          (const HashOpenMap<KEY,T,thash>& to_copy, double the_load_threshold = 1.0, int (*chash)(const KEY& a) = undefinedhash<KEY>);
    explicit HashOpenMap (const std::initializer_list<Entry>& il, double the_load_threshold = 1.0, int (*chash)(const KEY& a) = undefinedhash<KEY>);

    //Iterable class must support "for-each" loop: .begin()/.end() and prefix ++ on returned result
    template <class Iterable>
    explicit HashOpenMap (const Iterable& i, double the_load_threshold = 1.0, int (*chash)(const KEY& a) = undefinedhash<KEY>);


    //Queries
    bool empty      () const;
    int  size       () const;
    bool has_key    (const KEY& key) const;
    bool has_value  (const T& value) const;
    std::string str () const; //supplies useful debugging information; contrast to operator <<


    //Commands
    T    put   (const KEY& key, const T& value);
    T    erase (const KEY& key);
    void clear ();

    //Iterable class must support "for-each" loop: .begin()/.end() and prefix ++ on returned result
    template <class Iterable>
    int put_all(const Iterable& i);


    //Operators

    T&       operator [] (const KEY&);
    const T& operator [] (const KEY&) const;
    HashOpenMap<KEY,T,thash>& operator = (const HashOpenMap<KEY,T,thash>& rhs);
    bool operator == (const HashOpenMap<KEY,T,thash>& rhs) const;
    bool operator != (const HashOpenMap<KEY,T,thash>& rhs) const;

    template<class KEY2,class T2, int (*hash2)(const KEY2& a)>
    friend std::ostream& operator << (std::ostream& outs, const HashOpenMap<KEY2,T2,hash2>& m);



  public:
    class Iterator {
      public:
        //Private constructor called in begin/end, which are friends of HashOpenMap<T>
        ~Iterator();
        Entry       erase();
        std::string str  () const;
        HashOpenMap<KEY,T,thash>::Iterator& operator ++ ();
        HashOpenMap<KEY,T,thash>::Iterator  operator ++ (int);
        bool operator == (const HashOpenMap<KEY,T,thash>::Iterator& rhs) const;
        bool operator != (const HashOpenMap<KEY,T,thash>::Iterator& rhs) const;
        Entry& operator *  () const;
        Entry* operator -> () const;
        friend std::ostream& operator << (std::ostream& outs, const HashOpenMap<KEY,T,thash>::Iterator& i) {
          outs << i.str(); //Use the same meaning as the debugging .str() method
          return outs;
        }
        friend Iterator HashOpenMap<KEY,T,thash>::begin () const;
        friend Iterator HashOpenMap<KEY,T,thash>::end   () const;

      private:
        //If can_erase is false, must ++ to reach next value
        int                       current; //Bin Index
        HashOpenMap<KEY,T,thash>* ref_map;
        int                       expected_mod_count;
        bool                      can_erase = true;

        //Called in friends begin/end
        Iterator(HashOpenMap<KEY,T,thash>* iterate_over, int initial);
    };


    Iterator begin () const;
    Iterator end   () const;


private:
  int (*hash)(const KEY& k);      //Hashing function used (from template or constructor)
  enum bin_state {bs_empty, bs_occupied, bs_was_occupied};
  Entry* map       = nullptr;    //Entry array
  bin_state* state = nullptr;    //bin_state[] describes the state of map[i]
  double load_threshold;         //used/bins <= load_threshold
  int bins         = 1;          //# bins in array (should start at 1 so hash_compress doesn't % 0)
  int used         = 0;          //Cache for number of key->value pairs in the hash table
  int mod_count    = 0;          //For sensing concurrent modification


  //Helper methods
  int   hash_compress      (const KEY& key)  const;  //hash function ranged to [0,bins-1]
  int   find_key           (const KEY& key)  const;  //Returns index to key's bin or -1
  int   next_unoccupied    (int bin)         const;  //this bin or next beyond that is unoccupied

  void  ensure_load_threshold(int new_used);                 //Reallocate if load_factor > load_threshold
};




////////////////////////////////////////////////////////////////////////////////
//
//HashOpenMap class and related definitions

//Destructor/Constructors

template<class KEY,class T, int (*thash)(const KEY& a)>
HashOpenMap<KEY,T,thash>::~HashOpenMap() {
  delete [] map;
  delete [] state;
}


template<class KEY,class T, int (*thash)(const KEY& a)>
HashOpenMap<KEY,T,thash>::HashOpenMap(double the_load_threshold, int (*chash)(const KEY& k))
: hash(thash != (hashfunc)undefinedhash<KEY> ? thash : chash), load_threshold(the_load_threshold) {
  if (hash == (hashfunc)undefinedhash<KEY>)
    throw TemplateFunctionError("HashOpenMap::default constructor: neither specified");
  if (thash != (hashfunc)undefinedhash<KEY> && chash != (hashfunc)undefinedhash<KEY> && thash != chash)
    throw TemplateFunctionError("HashOpenMap::default constructor: both specified and different");
  if (the_load_threshold > 1.) {
    std::ostringstream answer;
    answer << "HashOpenMap::default constructor: the_load_threshold(" << the_load_threshold << ") > 1";
    throw IcsError(answer.str());
  }

  map   = new Entry    [bins];
  state = new bin_state[bins];
  for (int b=0; b<bins; ++b)
    state[b] = bs_empty;
}


template<class KEY,class T, int (*thash)(const KEY& a)>
HashOpenMap<KEY,T,thash>::HashOpenMap(int initial_bins, double the_load_threshold, int (*chash)(const KEY& k))
: hash(thash != (hashfunc)undefinedhash<KEY> ? thash : chash), bins(initial_bins), load_threshold(the_load_threshold) {
  if (hash == (hashfunc)undefinedhash<KEY>)
    throw TemplateFunctionError("HashOpenMap::length constructor: neither specified");
  if (thash != (hashfunc)undefinedhash<KEY> && chash != (hashfunc)undefinedhash<KEY> && thash != chash)
    throw TemplateFunctionError("HashOpenMap::length constructor: both specified and different");
  if (the_load_threshold > 1.) {
    std::ostringstream answer;
    answer << "HashOpenMap::length constructor: the_load_threshold(" << the_load_threshold << ") > 1";
    throw IcsError(answer.str());
  }

  if (bins < 1)
    bins = 1;
  map   = new Entry    [bins];
  state = new bin_state[bins];
  for (int b=0; b<bins; ++b)
    state[b] = bs_empty;
}


template<class KEY,class T, int (*thash)(const KEY& a)>
HashOpenMap<KEY,T,thash>::HashOpenMap(const HashOpenMap<KEY,T,thash>& to_copy, double the_load_threshold, int (*chash)(const KEY& a))
: hash(thash != (hashfunc)undefinedhash<KEY> ? thash : chash), load_threshold(the_load_threshold), bins(to_copy.bins) {
  if (hash == (hashfunc)undefinedhash<KEY>)
    hash = to_copy.hash;//throw TemplateFunctionError("HashOpenMap::copy constructor: neither specified");
  if (thash != (hashfunc)undefinedhash<KEY> && chash != (hashfunc)undefinedhash<KEY> && thash != chash)
    throw TemplateFunctionError("HashOpenMap::copy constructor: both specified and different");
  if (the_load_threshold > 1.) {
    std::ostringstream answer;
    answer << "HashOpenMap::length constructor: the_load_threshold(" << the_load_threshold << ") > 1";
    throw IcsError(answer.str());
  }

  if (hash == to_copy.hash && (double)to_copy.size()/to_copy.bins <= the_load_threshold) {
    used = to_copy.used;
    map   = new Entry    [bins];
    state = new bin_state[bins];
    for (int b = 0; b < bins; ++b) {
      state[b] = to_copy.state[b];
      if (state[b] == bs_occupied)
        map [b] = to_copy.map[b];
    }
  }else {
    bins = std::max(1,int(to_copy.size()/load_threshold));
    map   = new Entry    [bins];
    state = new bin_state[bins];
    for (int b=0; b<bins; ++b)
      state[b] = bs_empty;

    for (int b=0; b<to_copy.bins; ++b)
      if (to_copy.state[b] == bs_occupied)
        put(to_copy.map[b].first,to_copy.map[b].second);
  }
}


template<class KEY,class T, int (*thash)(const KEY& a)>
HashOpenMap<KEY,T,thash>::HashOpenMap(const std::initializer_list<Entry>& il, double the_load_threshold, int (*chash)(const KEY& k))
: hash(thash != (hashfunc)undefinedhash<KEY> ? thash : chash), load_threshold(the_load_threshold), bins(std::max(1,int(il.size()/the_load_threshold))) {
  if (hash == (hashfunc)undefinedhash<KEY>)
    throw TemplateFunctionError("HashOpenMap::initializer_list constructor: neither specified");
  if (thash != (hashfunc)undefinedhash<KEY> && chash != (hashfunc)undefinedhash<KEY> && thash != chash)
    throw TemplateFunctionError("HashOpenMap::initializer_list constructor: both specified and different");
  if (the_load_threshold > 1.) {
    std::ostringstream answer;
    answer << "HashOpenMap::initializer_list constructor: the_load_threshold(" << the_load_threshold << ") > 1";
    throw IcsError(answer.str());
  }

  map   = new Entry    [bins];
  state = new bin_state[bins];
  for (int b=0; b<bins; ++b)
    state[b] = bs_empty;

  for (const Entry& m_entry : il)
    put(m_entry.first,m_entry.second);
}


template<class KEY,class T, int (*thash)(const KEY& a)>
template <class Iterable>
HashOpenMap<KEY,T,thash>::HashOpenMap(const Iterable& i, double the_load_threshold, int (*chash)(const KEY& k))
: hash(thash != (hashfunc)undefinedhash<KEY> ? thash : chash), load_threshold(the_load_threshold), bins(std::max(1,int(i.size()/the_load_threshold))) {
  if (hash == (hashfunc)undefinedhash<KEY>)
    throw TemplateFunctionError("HashOpenMap::Iterable constructor: neither specified");
  if (thash != (hashfunc)undefinedhash<KEY> && chash != (hashfunc)undefinedhash<KEY> && thash != chash)
    throw TemplateFunctionError("HashOpenMap::Iterable constructor: both specified and different");
  if (the_load_threshold > 1.) {
    std::ostringstream answer;
    answer << "HashOpenMap::Iterable constructor: the_load_threshold(" << the_load_threshold << ") > 1";
    throw IcsError(answer.str());
  }

  map   = new Entry    [bins];
  state = new bin_state[bins];
  for (int b=0; b<bins; ++b)
    state[b] = bs_empty;

  for (const Entry& m_entry : i)
    put(m_entry.first,m_entry.second);
}


////////////////////////////////////////////////////////////////////////////////
//
//Queries

template<class KEY,class T, int (*thash)(const KEY& a)>
bool HashOpenMap<KEY,T,thash>::empty() const {
  return used == 0;
}


template<class KEY,class T, int (*thash)(const KEY& a)>
int HashOpenMap<KEY,T,thash>::size() const {
  return used;
}


template<class KEY,class T, int (*thash)(const KEY& a)>
bool HashOpenMap<KEY,T,thash>::has_key (const KEY& key) const {
  return find_key(key) != -1;
}


template<class KEY,class T, int (*thash)(const KEY& a)>
bool HashOpenMap<KEY,T,thash>::has_value (const T& value) const {
  for (int b=0; b<bins; ++b)
    if (state[b] == bs_occupied && value == map[b].second)
        return true;

  return false;
}


template<class KEY,class T, int (*thash)(const KEY& a)>
std::string HashOpenMap<KEY,T,thash>::str() const {
  std::ostringstream answer;
  answer << "HashOpenMap[";
  if (bins != 0) {
    answer << std::endl;
    for (int b=0; b<bins; ++b) {
      answer << "  bin[" << b << "] = ";
      switch (state[b]) {
        case bs_empty        : answer << "empty"                 << std::endl; break;
        case bs_occupied     : answer << "occupied : " << map[b].first << "->" << map[b].second << std::endl; break;
        case bs_was_occupied : answer << "was occupied : "       << std::endl; break;
      }
    }
  }
  answer  << "](load_threshold=" << load_threshold << ",bins=" << bins << ",used=" <<used <<",mod_count=" << mod_count << ")";
  return answer.str();
}


////////////////////////////////////////////////////////////////////////////////
//
//Commands

template<class KEY,class T, int (*thash)(const KEY& a)>
T HashOpenMap<KEY,T,thash>::put(const KEY& key, const T& value) {
  T to_return;
  int c = find_key(key);
  if (c != -1) {
    to_return = map[c].second;
    map[c].second = value;
  }else{
    to_return = value;
    ensure_load_threshold(used+1);
    ++used;
    int bin = next_unoccupied(hash_compress(key));
    state[bin] = bs_occupied;
    map  [bin] = Entry(key,value);
  }

  ++mod_count;
  return to_return;
}


template<class KEY,class T, int (*thash)(const KEY& a)>
T HashOpenMap<KEY,T,thash>::erase(const KEY& key) {
  int c = find_key(key);
  if (c == -1) {
    std::ostringstream answer;
    answer << "HashOpenMap::erase: key(" << key << ") not in Map";
    throw KeyError(answer.str());
  }
  T to_return = map[c].second;
  state[c] = bs_was_occupied;

  --used;
  ++mod_count;
  return to_return;
}


template<class KEY,class T, int (*thash)(const KEY& a)>
void HashOpenMap<KEY,T,thash>::clear() {
  for (int b=0; b<bins; ++b)
    state[b] = bs_empty;

  used = 0;
  ++mod_count;
}


template<class KEY,class T, int (*thash)(const KEY& a)>
template<class Iterable>
int HashOpenMap<KEY,T,thash>::put_all(const Iterable& i) {
  int count = 0;
  for (const Entry& m_entry : i) {
    ++count;
    put(m_entry.first, m_entry.second);
  }

  return count;
}


////////////////////////////////////////////////////////////////////////////////
//
//Operators

template<class KEY,class T, int (*thash)(const KEY& a)>
T& HashOpenMap<KEY,T,thash>::operator [] (const KEY& key) {
  int c = find_key(key);
  if (c != -1)
    return map[c].second;

  ensure_load_threshold(used+1);
  ++used;
  ++mod_count;
  int bin = next_unoccupied(hash_compress(key));     //may may have changed in ensure_load_threshold!
  state[bin] = bs_occupied;
  map  [bin] = Entry(key,T());
  return map[bin].second;
}


template<class KEY,class T, int (*thash)(const KEY& a)>
const T& HashOpenMap<KEY,T,thash>::operator [] (const KEY& key) const {
  int c = find_key(key);
  if (c != -1)
    return map[c].second;

  std::ostringstream answer;
  answer << "HashOpenMap::operator []: key(" << key << ") not in Map";
  throw KeyError(answer.str());
}


template<class KEY,class T, int (*thash)(const KEY& a)>
HashOpenMap<KEY,T,thash>& HashOpenMap<KEY,T,thash>::operator = (const HashOpenMap<KEY,T,thash>& rhs) {
  if (this == &rhs)
    return *this;

  if (hash == rhs.hash && (double)rhs.size()/rhs.bins <= load_threshold) {
    //Use a copy of the rhs data
    delete [] map;
    delete [] state;
    bins  = rhs.bins;
    used  = rhs.used;
    map   = new Entry    [bins];
    state = new bin_state[bins];

    for (int b=0; b<rhs.bins; ++b) {
      state[b] = rhs.state[b];
      if (state[b] == bs_occupied)
        map[b] = rhs.map[b];
    }
  }else {
    //Copy with rehashing
    delete[] map;
    delete[] state;
    bins = std::max(1,int(rhs.size()/load_threshold));
    map   = new Entry    [bins];
    state = new bin_state[bins];
    for (int b=0; b<bins; ++b)
      state[b] = bs_empty;

    for (int b=0; b<rhs.bins; ++b) {
      if (rhs.state[b] == bs_occupied)
        put(rhs.map[b].first, rhs.map[b].second);
    }
  }
  ++mod_count;
  return *this;
}


template<class KEY,class T, int (*thash)(const KEY& a)>
bool HashOpenMap<KEY,T,thash>::operator == (const HashOpenMap<KEY,T,thash>& rhs) const {
  if (this == &rhs)
    return true;
  if (used != rhs.size())
    return false;

  for (int b=0; b<bins; ++b)
    if (state[b] == bs_occupied) {
      // Uses ! and ==, so != on T need not be defined
      int rhs_c = rhs.find_key(map[b].first);
      if  (rhs_c == -1 || !(map[b].second == rhs.map[rhs_c].second))
        return false;
    }
    //More efficient than
    //if (state[b] == bs_occupied && (!rhs.has_key(map[b].first) || map[b].second != rhs[map[b].first]))
    //  return false;

  return true;
}


template<class KEY,class T, int (*thash)(const KEY& a)>
bool HashOpenMap<KEY,T,thash>::operator != (const HashOpenMap<KEY,T,thash>& rhs) const {
  return !(*this == rhs);
}


template<class KEY,class T, int (*thash)(const KEY& a)>
std::ostream& operator << (std::ostream& outs, const HashOpenMap<KEY,T,thash>& m) {
  outs << "map[";

  int printed = 0;
  for (int b=0; b<m.bins; ++b)
    if (m.state[b] == m.bin_state::bs_occupied) // (true) for debugging purposes
      outs << (printed++ == 0? "" : ",") << m.map[b].first << "->" << m.map[b].second;

  outs << "]";
  return outs;
}


////////////////////////////////////////////////////////////////////////////////
//
//Iterator constructors

template<class KEY,class T, int (*thash)(const KEY& a)>
auto HashOpenMap<KEY,T,thash>::begin () const -> HashOpenMap<KEY,T,thash>::Iterator {
  return Iterator(const_cast<HashOpenMap<KEY,T,thash>*>(this),0);
}


template<class KEY,class T, int (*thash)(const KEY& a)>
auto HashOpenMap<KEY,T,thash>::end () const -> HashOpenMap<KEY,T,thash>::Iterator {
  return Iterator(const_cast<HashOpenMap<KEY,T,thash>*>(this),bins);
}


////////////////////////////////////////////////////////////////////////////////
//
//Private helper methods

template<class KEY,class T, int (*thash)(const KEY& a)>
int HashOpenMap<KEY,T,thash>::hash_compress (const KEY& key) const {
  return abs(hash(key)) % bins;
}


template<class KEY,class T, int (*thash)(const KEY& a)>
int HashOpenMap<KEY,T,thash>::find_key (const KEY& key) const {
  int bin = hash_compress(key);
  int bin_original = bin;
  do {
    if (state[bin] == bs_empty)
      return -1;
    if (state[bin] == bs_occupied && map[bin].first == key)
        return bin;
    bin = ++bin%bins; //linear probing;
  } while (bin != bin_original);

  return -1;
}


template<class KEY,class T, int (*thash)(const KEY& a)>
int HashOpenMap<KEY,T,thash>::next_unoccupied (int bin) const {
  for (/*bin*/; state[bin] == bs_occupied; bin = ++bin%bins) //linear probing
    ;

  //load_threshold <= 1 guarantees something is empty/was_occupied
  return bin;
}


template<class KEY,class T, int (*thash)(const KEY& a)>
void HashOpenMap<KEY,T,thash>::ensure_load_threshold(int new_used) {
  if (double(new_used)/double(bins) <= load_threshold)
    return;
  int        old_bins  = bins;
  Entry*     old_map   = map;
  bin_state* old_state = state;

  bins  = 2*old_bins;
  used  = 0;
  map   = new Entry    [bins];
  state = new bin_state[bins];

  for (int b=0; b<bins; ++b)
    state[b] = bs_empty;

  for (int b=0; b<old_bins; ++b)
    if (old_state[b] == bs_occupied)
      put(old_map[b].first,old_map[b].second);

  delete [] old_map;
  delete [] old_state;
}






////////////////////////////////////////////////////////////////////////////////
//
//Iterator class definitions

template<class KEY,class T, int (*thash)(const KEY& a)>
HashOpenMap<KEY,T,thash>::Iterator::Iterator(HashOpenMap<KEY,T,thash>* iterate_over, int initial)
: ref_map(iterate_over), expected_mod_count(ref_map->mod_count) {
  current = initial;
  while (current < ref_map->bins && ref_map->state[current] != bs_occupied)
     ++current;
}


template<class KEY,class T, int (*thash)(const KEY& a)>
HashOpenMap<KEY,T,thash>::Iterator::~Iterator()
{}


template<class KEY,class T, int (*thash)(const KEY& a)>
auto HashOpenMap<KEY,T,thash>::Iterator::erase() -> Entry {
  if (expected_mod_count != ref_map->mod_count)
    throw ConcurrentModificationError("HashOpenMap::Iterator::erase");
  if (!can_erase)
    throw CannotEraseError("HashOpenMap::Iterator::erase Iterator cursor already erased");
  if (current < 0 || current >= ref_map->bins)
    throw CannotEraseError("HashOpenMap::Iterator::erase Iterator cursor beyond data structure");

  can_erase = false;
  Entry to_return = ref_map->map[current];
  ref_map->state[current] = bs_was_occupied;

  --ref_map->used;
  ++ref_map->mod_count;
  expected_mod_count = ref_map->mod_count;

  return to_return;
}


template<class KEY,class T, int (*thash)(const KEY& a)>
std::string HashOpenMap<KEY,T,thash>::Iterator::str() const {
  std::ostringstream answer;
  answer << ref_map->str() << "(current=" << current << ",expected_mod_count=" << expected_mod_count << ",can_erase=" << can_erase << ")";
  return answer.str();
}


template<class KEY,class T, int (*thash)(const KEY& a)>
auto  HashOpenMap<KEY,T,thash>::Iterator::operator ++ () -> HashOpenMap<KEY,T,thash>::Iterator& {
  if (expected_mod_count != ref_map->mod_count)
    throw ConcurrentModificationError("HashOpenMap::Iterator::operator ++");

  if (current >= ref_map->bins)
    return *this;

  for (++current; current < ref_map->bins && ref_map->state[current] != bs_occupied; ++current)
    ;

  can_erase = true;

  return *this;
}


template<class KEY,class T, int (*thash)(const KEY& a)>
auto  HashOpenMap<KEY,T,thash>::Iterator::operator ++ (int) -> HashOpenMap<KEY,T,thash>::Iterator {
  if (expected_mod_count != ref_map->mod_count)
    throw ConcurrentModificationError("HashOpenMap::Iterator::operator ++(int)");

  Iterator to_return(*this);

  if (current >= ref_map->bins)
    return to_return;

  for (++current; current < ref_map->bins && ref_map->state[current] != bs_occupied; ++current)
    ;

  can_erase = true;

  return to_return;
}


template<class KEY,class T, int (*thash)(const KEY& a)>
bool HashOpenMap<KEY,T,thash>::Iterator::operator == (const HashOpenMap<KEY,T,thash>::Iterator& rhs) const {
  const Iterator* rhsASI = dynamic_cast<const Iterator*>(&rhs);
  if (rhsASI == 0)
    throw IteratorTypeError("HashOpenMap::Iterator::operator ==");
  if (expected_mod_count != ref_map->mod_count)
    throw ConcurrentModificationError("HashOpenMap::Iterator::operator ==");
  if (ref_map != rhsASI->ref_map)
    throw ComparingDifferentIteratorsError("HashOpenMap::Iterator::operator ==");

  return this->current == rhsASI->current;
}


template<class KEY,class T, int (*thash)(const KEY& a)>
bool HashOpenMap<KEY,T,thash>::Iterator::operator != (const HashOpenMap<KEY,T,thash>::Iterator& rhs) const {
  const Iterator* rhsASI = dynamic_cast<const Iterator*>(&rhs);
  if (rhsASI == 0)
    throw IteratorTypeError("HashOpenMap::Iterator::operator !=");
  if (expected_mod_count != ref_map->mod_count)
    throw ConcurrentModificationError("HashOpenMap::Iterator::operator !=");
  if (ref_map != rhsASI->ref_map)
    throw ComparingDifferentIteratorsError("HashOpenMap::Iterator::operator !=");

  return this->current != rhsASI->current;
}


template<class KEY,class T, int (*thash)(const KEY& a)>
pair<KEY,T>& HashOpenMap<KEY,T,thash>::Iterator::operator *() const {
  if (expected_mod_count != ref_map->mod_count)
    throw ConcurrentModificationError("HashOpenMap::Iterator::operator *");
  if (!can_erase || current < 0 || current >= ref_map->bins)
    throw IteratorPositionIllegal("HashOpenMap::Iterator::operator * Iterator illegal");

  return ref_map->map[current];
}


template<class KEY,class T, int (*thash)(const KEY& a)>
pair<KEY,T>* HashOpenMap<KEY,T,thash>::Iterator::operator ->() const {
  if (expected_mod_count != ref_map->mod_count)
    throw ConcurrentModificationError("HashOpenMap::Iterator::operator ->");
  if (!can_erase || current < 0 || current >= ref_map->bins)
    throw IteratorPositionIllegal("HashOpenMap::Iterator::operator -> Iterator illegal");

  return &(ref_map->map[current]);
}


}

#endif /* HASH_OPEN_MAP_HPP_ */
