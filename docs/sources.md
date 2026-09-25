# Supplier references and geometry provenance

Retrieved and reviewed during development on **7 September 2026**. Availability, prices and supplier documentation may change. Prices are not embedded as purchase recommendations.

## Tubeclamp AU

The [premade-kits gallery](https://www.tubeclamp.com.au/pages/premade-kits) was inspected across its creation photographs. It includes garment displays, monkey bars/rings, an up-and-over mezzanine gate, timber tables, trolley barriers, wall and freestanding shelving, hinged barriers, handrails, a wheeled kitchen trolley, timber-supported outdoor structures and a playground frame. These examples informed the need for world mounts, panels, wheels, swivels and human motion, rather than just a lattice of pipe endpoints.

The local [gallery manifest](../sources/inspiration/gallery-sources.json) records the image URLs and available captions. Contact sheets are in `sources/inspiration/gallery-*.jpg`. Gallery photos are references; the supplied example designs are original small demonstrations, not reverse-engineered commercial kits.

Catalogue families reconstructed from the supplier's published dimension images:

| Code | Interface represented | Supplier |
|---|---|---|
| TC101 | Through run, terminal branch | [Short tee](https://www.tubeclamp.com.au/products/tc101) |
| TC104 | Two terminal run sockets and branch | [Long tee](https://www.tubeclamp.com.au/products/tc104) |
| TC116 | Continuous upright and two terminal rails at 90°; T/A/B/C/D sizes | [Corner middle elbow](https://www.tubeclamp.com.au/products/tc116) |
| TC125 | Two terminal sockets at 90° | [Elbow](https://www.tubeclamp.com.au/products/tc125) |
| TC128 | Three terminal corner sockets | [Corner top elbow](https://www.tubeclamp.com.au/products/tc128) |
| TC131 | Tube socket and mounting bolt ports | [Round flange](https://www.tubeclamp.com.au/products/tc131) |
| TC132 | Tube socket and railing-base bolt ports | [Railing base](https://www.tubeclamp.com.au/products/tc132) |
| TC136 | Radially installable split through fitting | [Split tee](https://www.tubeclamp.com.au/products/tc136) |
| TC138 | Through socket and hinge eye | [Gate eye](https://www.tubeclamp.com.au/products/tc138) |
| TC140 | Through socket and hinge pin | [Gate hinge pin](https://www.tubeclamp.com.au/products/tc140) |
| TC148 | Short through collar and stepped terminal branch; A/B/C/D/E sizes | [Swivel short tee](https://www.tubeclamp.com.au/products/tc148) |
| TC161 | Two perpendicular, offset through sockets; A/B/C/D/E sizes | [Standard crossover](https://www.tubeclamp.com.au/products/tc161) |
| TC173M | Tube socket and male swivel plate | [Male swivel](https://www.tubeclamp.com.au/products/tc173m) |
| TC173F | Tube socket and female swivel interface | [Female swivel](https://www.tubeclamp.com.au/products/tc173f) |
| TC179 | Through-bore locking stop | [Locking ring](https://www.tubeclamp.com.au/products/tc179) |

Sizes use actual tube OD: T 21.3, A 26.9, B 33.7, C 42.4, D 48.3 and E 60.3 mm. Only sizes transcribed from each family's dimension table are included. See the [fitting size guide](https://www.tubeclamp.com.au/pages/fitting-size-guide).

Each source directory has the supplier product URL, original image URL, retrieval timestamp and SHA-256 of downloaded drawings. `drawing_dimensions_mm` preserves the supplier's f/g/h/etc labels. `scripts/make_library.py` makes the local primitive models from those data. Every part retains its geometry status and assumptions.

TC161 uses local Z for `through` and local X for `cross`, separated along Y by the drawing's `i`. Both sockets accept continuous pipes. For C42, f=g=45.1, h=106.4 and i=42.4 mm; drawing mass is 0.490 kg. The two bores are subtracted from the combined casting, preserving both openings where the shells overlap. The packaged STL and its convex collision compound represent the same clear bores; the casting blend and screw bosses remain approximations.

TC148 was added on 8 September 2026. Its [drawing and reference photos](../sources/tubeclamp/tc148/source.json) describe one rigid casting, commonly paired on a shared pipe to make an adjustable 60–180° corner/cross. Local Z is the short continuous collar; the terminal X socket sits at its collar face. Two inverted collars can stack with their branches at the same height. C42 uses f=28.5, g=101.3, h=49.9, i=27 mm and drawing mass 0.438 kg. The stepped mesh has open bores and a separate collision compound. The tapered web, casting walls, axial branch offset and contact stops are approximations. The published g envelope conflicts with h+i and the bore radius for D/E, so the model uses the documented h+i reach and an assumed OD+9 casting diameter rather than silently altering the supplier table. See the [paired example](../examples/swivel-short-tee.pipe.yaml).

TC136 now has its own split-cap mesh, bolt ears, clamp bolts and terminal socket envelope. Both bores accept the nominal tube OD with the library's declared clearance; a C42 split tee is not a reducing fitting. Its existing port positions and slide/radial installation rules are preserved. The f/g/h table remains intact. The drawing does not fully specify the casting contour or split hardware; their approximations and the use of g for the ear span are recorded in the part's assumptions. These are assembled fittings: the cap is shown separately in the mesh but is not an independently simulated body.

TC104 supports both a continuous main pipe (`through`) and two separate pipe ends (`run_start`, `run_end`), as confirmed by the supplier's description of its unobstructed main bore. The human-editable port `excludes` lists prevent these alternative uses sharing that bore at once. The continuous port is centred on the fitting and uses `at_mm` along the member; insertion depth is reserved for end sockets.

TC116 was added on 8 September 2026 (Adelaide time). Its local `through` socket follows Z; `x` and `y` are terminal sockets for the two horizontal rails. The continuous bore is modelled as one socket, accepting one unbroken upright, with sliding and rotation when its screw is loose. The C42 drawing gives f=88.9 mm and mass=0.634 kg. The casting diameter, through engagement and rail engagement are inferred and labelled like the other fittings. The [supplier drawing](https://cdn.shopify.com/s/files/1/1384/6879/files/TC116TechnicalDiagram-2025.jpg?v=1750743058) lists f=61.5 mm for E60, inconsistent with a 60.3 mm bore and terminal rail engagement; E60 is withheld pending a corrected drawing. The separately listed TC116ZD clamp-on variant is also withheld because its dimensions are not provided. Its radial installation must not be attributed to the standard slip-on TC116.

For example, the TC101 C-size table gives f=56, g=89, h=59 mm and 0.50 kg. The TC132 C-size table gives f=86, g=10, h=138, i=101, j=80, k=15 mm and 1.15 kg. Drawing mass is used when it differs from storefront/shipping metadata. This distinction is recorded.

Unsupplied casting contours, fillets, grub-screw heads, exact socket bore clearance and screw engagement are approximated. Bolt-hole centres are annotated as ports; visual flange envelopes do not contain all detailed holes. Tube OD is sourced, while default wall thickness and steel grade are explicit assumptions. The 5,650 N family axial-slip statement at 40 N·m is conditional and not a per-mode casting fracture specification.

## Aluminium framing and belts

- The user's [MiniTec enclosure examples](https://www.minitecframing.com/application_pages/Enclosures_2016/Machine_Enclosures_Lab_Enclosures_Mini-Environments.html) establish profile frames with panels and equipment loads.
- [MiniTec profile 20.1006](https://www.minitecframing.com/Products/Aluminum_Profiles/Aluminum_Profile_Catalog_Pages/20.1006_Aluminum_Profile_45x45.html): 45 × 45 mm, bending inertias 15.934 cm⁴ and mass 2.192 kg/m. The visual slot shape is simplified; FEA uses published section properties. [UK profile data](https://shop.minitec.co.uk/product/45-x-45-profile/) supplies the torsional constant; its different 2.205 kg/m mass is noted rather than silently substituted.
- [MiniTec fastener catalogue](https://www.minitecframing.com/PDF/Catalog_Sections/MiniTec_Fasteners.pdf) identifies Power-Lock 21.1018/0, M8 × 25 hardware and mass 0.029 kg. Its small element envelope is estimated. [Assembly and technical data](https://www.minitecframing.com/Products/Profile_Fasteners/Profile_Fasteners_PDF/MiniTec_Fastener_Technical_Data.pdf) describes end tapping, sliding the mating profile and 12 N·m locking. Its static-load statement is retained as contextual source data, not treated as arbitrary-direction capacity.
- [Adafruit pulley 1251](https://www.adafruit.com/product/1251): GT2, 20 teeth, 5 mm bore, approximately 16 mm diameter/length and 6 g. This is archived hardware; availability is not implied. Teeth are represented by a pitch relationship, not individual contact geometry.
- [Adafruit belt 1184](https://www.adafruit.com/product/1184): 2 mm pitch, 6 mm width and 1,164 mm stock length. Thickness/mass are approximate. A drive's route and coupling describe its installed use.

The supplied [AliExpress item](https://www.aliexpress.com/item/1005004200751650.html) could not be accessed during research. Its geometry and specifications were not invented or attributed to the catalogue.

## Timber and printed connectors

- [Porta DOW-19](https://www.porta.com.au/products/mouldings/dowels/dowel-19): 19 mm dowel, available in Clear Pine/Eucalyptus Grandis with 1.2, 1.8, 2.4 and 3 m listed lengths. The library selects Eucalyptus Grandis geometry with explicitly assumed material properties.
- [Porta DAR-3018](https://www.porta.com.au/products/mouldings/dressed-boards/dar-3018): 30 × 18 mm dressed timber, with 1.2, 1.8, 2.4, 2.7 and 3 m listed lengths. These are dimensioned structural members in the beam model.
- The [18 mm Printables connector](https://www.printables.com/model/1320655-connectors-for-18mm-tubedowel) could not be retrieved. `generic.dowel-elbow-18` is an original illustrative envelope, explicitly labelled as such. Import the actual mesh and its licence to use that design. The 19 mm Porta dowel is deliberately not marked compatible with an 18 mm socket.

## Human reference basis

[De Leva's segment-parameter work](https://pubmed.ncbi.nlm.nih.gov/8872282/) and [NASA's Human Integration Design Handbook](https://www.nasa.gov/wp-content/uploads/2023/03/human-integration-design-handbook-revision-1.pdf) are background references for segmentation and anthropometric fit. The shipped masses, geometric proportions and ranges are declared engineering defaults, not reproduced tables or a validated implementation of either dataset. Record the measurements and intended population when calibrating a mannequin.
