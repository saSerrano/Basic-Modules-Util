# Basic Modules Utilities

El propósito de este repositorio es el de proveer un formato estándar para el desarrollo de habilidades del robot de servicio Markovito, al cual nos referimos como **módulo básico**.
Encapsular un conjunto de habilidades relacionadas entre sí en un **módulo básico** ofrece los siguientes beneficios:
- Facilita la integración de diversas habilidades (que no están relacionadas entre sí) en un solo sistema.
- Permite el desarrollo de distintas habilidades de manera simultánea.
- Al seguir un mismo formato base, la documentación de las habilidades resulta sencilla de entender para aquellas personas que no desarrollaron dicha habilidad.

## Documentación y archivos plantilla
En los directorios ```src``` y ```include/basicmodutil_pkg``` se encuentran archivos plantilla con los cuales uno puede desarrollar un **módulo básico**, ya sea python o C++.
Por otra parte, en el directorio ```utils``` se encuentran los siguientes archivos:
- ```Basic_Module_Handbook.pdf```: archivo PDF que contiene la documentación para el uso de los archivos plantilla contenidos en este repositorio. En este archivo se detalla la motivación para encapsular habilidades en **módulos básicos**, así como varios ejemplos a partir de los archivos plantilla.
- ```writeBM.py```: un script que facilita la generación de archivos JSON que son necesarios para la documentación de los **módulos básicos**.
- ```DEMO_BM.json``` y ```PYTHON_BM.json```: ejemplos de archivos JSON de documentación para **módulos básicos**.
