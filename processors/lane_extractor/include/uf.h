class UF    {
    int *id, cnt, *sz, capacity;
public:
	// Create an empty union find data structure with N isolated sets.
    UF(int N)   {
        capacity = N;
        cnt = N;
	id = new int[N];
	sz = new int[N];
        for(int i=0; i<N; i++)	{
            id[i] = i;
	    sz[i] = 1;
	}
    }
    ~UF()	{
	delete [] id;
	delete [] sz;
    }
	// Return the id of component corresponding to object p.
    int find(int p)	{
        int root = p;
        while (root != id[root])
            root = id[root];
        while (p != root) {
            int newp = id[p];
            id[p] = root;
            p = newp;
        }
        return root;
    }
	// Replace sets containing x and y with their union.
    void merge(int x, int y)	{
        int i = find(x);
        int j = find(y);
        if (i == j) return;
		
		// make smaller root point to larger one
        if   (sz[i] < sz[j])	{ 
		id[i] = j; 
		sz[j] += sz[i]; 
	} else	{ 
		id[j] = i; 
		sz[i] += sz[j]; 
	}
        cnt--;
    }
	// Are objects x and y in the same set?
    bool connected(int x, int y)    {
        return find(x) == find(y);
    }
	// Return the number of disjoint sets.
    int count() {
        return cnt;
    }

    // Returns the id of the largest set
    int max() {
      int maxSz = 0;
      int maxI = -1;
      for (int i = 0; i < capacity; ++i) {
         if (sz[i] > maxSz) {
           maxSz = sz[i];
           maxI = i;
         }
      }
      return maxI;
    }

    // Return all objects that are in the set containing p.
    int get(int p, int *object, int objCount) {
      int root = find(p);
      int out = 0;
      for (int i = 0; i < capacity && out < objCount; ++i) {
        if (find(i) == root) {
          object[out++] = i;
        }
      }
      return out;
    }
};
