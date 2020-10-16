// general utilities for use in the GUI

// given an object and a test function, return a new object where only
// the values where test(value) == true are kept, as well as the parents
// of those objects. Nested non-matching children are set to {}
// ex: obj = {parent: {child1: {child2: "hello"}, child1a: {child2a: "world"}}}, test = ((o) => o == "hello"),
// filterBy(obj, test) -> {parent: {child1: {child2: "hello"}, child1a: {}}}

export let filterBy = (obj, test) => {
    let retobj = {};
    Object.entries(obj).forEach(([key, val]) => {
        if (test(val)) {
            retobj[key] = val;
        }
        else if (typeof val == 'object' && val != null) {
            retobj[key] = filterBy(val, test);
        }
    })
    return retobj;
}

// given an object and test function, delete values where test(val) == true
// modifies obj in place
export let deleteBy = (obj, test) => {
    Object.keys(obj).forEach((key) => {
        if (typeof obj[key] == 'object' && obj[key] != null) {
            deleteBy(obj[key], test);
        }
        if (test(obj[key])) {
            delete obj[key];
        }
    })
}