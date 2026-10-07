// Run before opening tests/human-model-render.html on a local static server.
import {mkdir,writeFile,access} from 'node:fs/promises';
import {makeRig,exportRig} from '../tests/helpers/human-rig-fixtures.mjs';
import {humanPhysicsFixture} from '../tests/helpers/human-physics-fixture.mjs';
const directory='output/human-model-regression';await mkdir(directory,{recursive:true});
for(const variant of ['mixamo','blender'])await writeFile(`${directory}/${variant}.glb`,Buffer.from(await exportRig(makeRig({variant}))));
const models=[['Synthetic Mixamo',`/${directory}/mixamo.glb`],['Synthetic Blender',`/${directory}/blender.glb`]];
// Optional reference assets from three.js/examples/models/gltf are kept outside
// version control. Their absence does not make the offline regression fail.
for(const name of ['Soldier','Xbot']){const path=`output/human-model-checks/${name}.glb`;try{await access(path);models.push([`${name} (three.js example)`,'/'+path]);}catch{}}
await writeFile(`${directory}/models.json`,JSON.stringify(models));
// Use the real generator, including body-centred part origins and offset pivots.
await writeFile(`${directory}/poses.json`,JSON.stringify(humanPhysicsFixture()));
console.log(`Generated two weighted GLBs, six mannequin poses and three simulation frames; gallery has ${models.length} models in ${directory}`);
