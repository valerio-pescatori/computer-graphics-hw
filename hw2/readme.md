# Yocto/Raytrace: Tiny Raytracer

In questo homework, impareremo i principi di base della sintesi di immagini 
implementando un semplice path tracer. In particolare, impareremo come

- usare funzioni di intersezione di raggi,
- scrivere semplici shaders,
- scrivere un semplice renderer

## Framework

Il codice utilizza la libreria [Yocto/GL](https://github.com/xelatihy/yocto-gl),
che è inclusa in questo progetto nella directory `yocto`. 
Si consiglia di consultare la documentazione della libreria che si trova 
all'inizio dei file headers. Inoltre, dato che la libreria verrà migliorata 
durante il corso, consigliamo di mettere star e watch su github in modo che 
arrivino le notifiche di updates. In particolare, includiamo

- **yocto_math.h**: libreria di funzioni matematiche e di basso livello come;
- **yocto_image.{h,cpp}**: libreria che definisce l'oggetto immagine e permette 
  il caricamento e il salvataggio delle immagini;
- **yocto_commonio.h**: libreria per lo sviluppo di applicazioni a riga di comando;
- **yocto_gui.{h,cpp}**: libreria per lo sviluppo di semplici interfacce 
  utente.

Il codice è compilabile attraverso [Xcode](https://apps.apple.com/it/app/xcode/id497799835?mt=12)
su OsX e [Visual Studio 2019](https://visualstudio.microsoft.com/it/vs/) su Windows, 
con i tools [cmake](www.cmake.org) e [ninja](https://ninja-build.org) 
come mostrato in classe e riassunto, per OsX, 
nello script due scripts `scripts/build.sh`.
Per compilare il codice è necessario installare Xcode su OsX e 
Visual Studio 2019 per Windows, ed anche i tools cmake e ninja.
Come discusso in classe, si consiglia l'utilizzo di 
[Visual Studio Code](https://code.visualstudio.com), con i plugins 
[C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools) e
[CMake Tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools) 
extensions, che è già predisposto per lavorare su questo progetto.

Questo homework consiste nello sviluppare vari shaders che, per semplicità,
sono tutti contenuti nel file `yocto_raytrace.cpp`. Gli shader che scriverete
sono eseguiti per ogni pixel dell'immagine dalla funzione `trace_samples()`
che diamo già implementata. A sua volta questa funzione è chiamata da 
`yscenetrace.cpp` come interfaccia a riga di comando, e da `ysceneitraces.cpp` 
che mostra una semplice interfaccia grafica.

Questo repository contiene anche tests che sono eseguibili da riga di comando 
come mostrato in `run.sh`. Le immagini generate dal runner sono depositate 
nella directory `out/`. Questi risultati devono combaciare con le immagini nella 
directory `check/`. Per la consegna dell'homework basta renderizzare solo le
immagini a bassa risoluzione che andranno nella directory `out/lowres`.
In `check/highres` ci sono immagini calcolate ad alta risoluzione per farvi
vedere il miglioramento. 

## Funzionalità

In this homework you will implement the following features:

- **Color Shader** nella funzione `trace_color()`:
    - implementare uno shader che ritorna il colore del materiale del punto di intersezione
    - use `intersect_scene_bvh()` for intersection
- **Normal Shader** nella funzione `trace_normal()`:
    - implementare uno shader che ritorna la normale del punto di intersezione,
      tradotta in colore aggiungendo 0.5 e moltiplicando per 0.5
- **Texcoord Shader** nella funzione `trace_texcoord()`:
    - implementare uno shader che ritorna le texture coordinates del punto di intersezione,
      tradotte in colori per i canali RG; usare la funzione `fmod()` per forzarle 
      nel range [0, 1]
- **Eyelight Shader** nella funzione `trace_eyelight()`:
    - implementare uno shader che calcola il diffuse shading assumendo di avere 
      una fonte di illuminazione nelle fotocamera
- **Raytrace Shader** nella funzione `trace_raytrace()`:
    - implementare una shader ricorsivo che simula l'illuminazione per una varietà 
      di materiali seguendo gli steps descritti nelle lecture notes riassunti qui
      di seguito
    - calcolare posizione, normale and texture coordinates
    - calcolare il valore dei materiali multiplicando le constanti trovate 
      nel material con i valori nelle textures
    - implementare i seguenti materiali nell'ordinane presentato nelle slides: 
      polished transmission, polished metals, rough metals, rough plastic, and matte
    - potete usare tutte le funzioni di Yocto/Math ed in particolare `math::fresnel_schlick()`,
      `math::microfacet_distribution()` and `math::microfacet_shadowing()` 

## Extra Credit

- **WYOS**, write your own shader:
    - scrivere shader aggiuntivi e provateli su alcuni modelli
    - esempio: [cell shading](https://roystan.net/articles/toon-shader.html)
    - esempio: [matcap shading 1](http://viclw17.github.io/2016/05/01/MatCap-Shader-Showcase/), 
               [matcap shading 2](https://github.com/hughsk/matcap)
- **MYOI**, make your own image: 
    - creare immagini aggiuntive con il vostro renderer da modelli assemblati da voi
    - per creare modelli nuovi, dovete editare i file json che sono la serializzazione
      delle variabili negli oggi di Yocto/SceneIO
    - il vostro renderer fa molto bene environment maps e materiali diffusi 
      quindi io mi concentrerei du quelli
          - trovate ottime environment maps su [HDRIHaven](https://hdrihaven.com)
    - come punto di partenza potete assemblare nuove scene mettendo nuovi 
      modelli 3D, textures e environment maps
          - per le textures, cercate "free PBR textures" su Google as esempio
          - per i modelli non so bene
    - oppure potete provare a convertire modelli glTF o OBJ, magari Blender; 
      ma notate che in generale luci e materiali non sono esportati 

## Istruzioni

Per consegnare l'homework è necessario inviare una ZIP che include il codice e 
le immagini generate, cioè uno zip _con le sole directories `yocto_raytrace` e `out`_.
Per chi fa l'extra credit, includere anche `apps` e ulteriori immagini.
Per chi crea nuovi modelli, includere anche le directory con i nuovo modelli creati.
Il file va chiamato `<cognome>_<nome>_<numero_di_matricola>.zip` 
e vanno escluse tutte le altre directory. Inviare il file su Google Classroom.
