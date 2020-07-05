function makePromise() {
  let accept, reject;
  const p = new Promise((a, r) => {
    accept = a;
    reject = r;
  });
  p.accept = accept;
  p.reject = reject;
  return p;
}
const promise = makePromise();
export {promise};

globalThis.wasmModule = (moduleName, moduleFn) => {
  if (moduleName === 'vxl') {
    globalThis.Module = moduleFn({
      print(text) { console.log(text); },
      printErr(text) { console.warn(text); },
      locateFile() {
        return 'bin/objectize.wasm';
      },
      onRuntimeInitialized: () => {
        promise.accept();
      },
    });
  } else {
    console.warn('unknown wasm module', moduleName);
  }
};
