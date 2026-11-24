"""Write the device stream from a split capture as replay input: (f64 t, u32 len, bytes)*."""
import pickle, struct, sys
cap = pickle.load(open(sys.argv[1], 'rb'))
with open(sys.argv[2], 'wb') as f:
    for t, d in cap['reads']:
        f.write(struct.pack('<dI', t, len(d)) + d)
