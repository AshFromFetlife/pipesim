import {mkdir,cp} from 'node:fs/promises';
import {resolve} from 'node:path';
const root=resolve('pipesim/web/vendor');
await mkdir(root,{recursive:true});
for(const name of ['three.module.js','three.core.js']) await cp(resolve('node_modules/three/build',name),resolve(root,name));
for(const name of ['controls/OrbitControls.js','controls/TransformControls.js','loaders/GLTFLoader.js','loaders/STLLoader.js','loaders/OBJLoader.js','utils/BufferGeometryUtils.js']){
  await mkdir(resolve(root,'addons',name,'..'),{recursive:true});
  await cp(resolve('node_modules/three/examples/jsm',name),resolve(root,'addons',name));
}
await cp(resolve('node_modules/three/LICENSE'),resolve(root,'LICENSE-three.txt'));
console.log('Three.js assets copied for offline use.');
