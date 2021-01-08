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

// sets up a timeout to wait for fun() to evaluate to true;
// fun must return {done: boolean, data: <whatever>}
export let waitFor = (fun) => {
    return new Promise(resolve => {
        let check = () => {
            let res = fun();
            if (res.done) {
                resolve(res.data);
            }
            else {
                setTimeout(check,100);
            }
        }
        check();
    });
}

// utility function to allow time for things to happen
// async wait
export let wait = (time) => {
    return new Promise(resolve => {
        setTimeout(() => resolve(), time);
    });
}

export let pathsFromTree = (tree) => {
    let flatpaths = [];
    let path = [];
    let pathFromTree = (tree) => {
        for (const key of Object.keys(tree)) {
            path.push(key);
            if (tree[key] != null && typeof tree[key] == "object") {
              pathFromTree(tree[key]);
            }
            else {
              flatpaths.push(path.join('.'));
            }
            path.pop();
        }
    }
    // array path
    pathFromTree(tree);
    return flatpaths;
}

export let numberDisplay = (val) => {
    // if a number can be represented with 3 decimals, return it in that form
    // otherwise, return scientific notation
    let retVal = '';
    try {
        retVal = parseFloat(val).toFixed(3);
        if (retVal == '0.000' && val != 0 || retVal.length > 7) {
            retVal = parseFloat(val).toExponential(3);
        }
    } catch (error) {
        console.log(error);
    }
    return retVal;
}