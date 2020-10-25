<!-- TABLE OF CONTENTS -->
## A Multi-Robot GSPN Software Framework to Execute and Visualize Plans
This package was developed in the scope of my Master's Thesis in Instituto Superior Técnico. The package includes two different implementations, diferentiated by the technologies that are supported. Inside the "common" folder, you will find the standalone version of our framework where you can use a GSPN tool to execute and visualize plans in a system with agents. Inside the "ros" folder, you will find the ROS implementation of our framework where you can use a GSPN tol to execute and visualize plans in a system with robots.  

## Table of Contents

* [About the Project](#about-the-project)
  * [Built With](#built-with)
* [Getting Started](#getting-started)
  * [Prerequisites](#prerequisites)
  * [Installation](#installation)
* [Usage](#usage)
* [Roadmap](#roadmap)
* [Contributing](#contributing)
* [License](#license)
* [Contact](#contact)
* [Acknowledgements](#acknowledgements)



<!-- ABOUT THE PROJECT -->
## About The Project

[![Product Name Screen Shot][product-screenshot]](https://example.com)

This package extended the package created on https://github.com/cazevedo/gspn-framework by adding two new modules: the execution module and the visualization module. The architecture is briefly presented in . And more specifically in .

### Built With
This framework was generally built with concurrent futures, Vis.js, sparse, numpy, flask and json. 
* [Concurrent futures](https://docs.python.org/3/library/concurrent.futures.html#module-concurrent.futures)
* [Vis.js](https://visjs.org/)
* [Sparse](https://pypi.org/project/sparse/)
* [Numpy](https://numpy.org/)
* [Flask](https://flask.palletsprojects.com/en/1.1.x/)
* [Json](https://www.json.org/json-en.html)

The ROS implementation was built with:



<!-- GETTING STARTED -->
## Getting Started

This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.

### Prerequisites

This is an example of how to list things you need to use the software and how to install them.
* npm
```sh
npm install npm@latest -g
```

### Installation

1. Get a free API Key at [https://example.com](https://example.com)
2. Clone the repo
```sh
git clone https://github.com/your_username_/Project-Name.git
```
3. Install NPM packages
```sh
npm install
```
4. Enter your API in `config.js`
```JS
const API_KEY = 'ENTER YOUR API';
```



<!-- USAGE EXAMPLES -->
## Standalone version usage 
Before running the example for the standalone version, you should change the "project_path" in the example JSON input file (gspn_execution_input.json) because that path is set to my computer and it will not work in yours. 
After doing so, you can either run the full standalone version (visualization + execution) or simply the execution module. 
An important note to take is that since the used GSPN has no immediate transitions, "places_tuple" and "policy_dictionary" are not considered. However, those values are set for illustrative reasons. 
THe input file uses a very simple GSPN with three places and two exponential transitions. The functions used are also very simple as well (they only print a message into the terminal).  

In order to run the full standalone version, all you need to do is run the following command inside gspn_framework_package/common/src/gspn_framework_package/:
```sh
python3 gspn_visualization.py
```
Afterwards, click on the link on the terminal next to "Running on", choose the example JSON input and play around with the user interface. 

However, if you only want to run the execution module, you must run the following command inside gspn_framework_package/common/src/gspn_framework_package/:
```sh
python3 gspn_execution.py
```

On this case, you will be queried about the whereabouts of the input file and you can provide the path to the example JSON input file. 

<!-- USAGE EXAMPLES -->
## ROS version usage 



<!-- ROADMAP -->
## Roadmap

See the [open issues](https://github.com/othneildrew/Best-README-Template/issues) for a list of proposed features (and known issues).



<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to be learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request



<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE` for more information.



<!-- CONTACT -->
## Contact

Your Name - [@your_twitter](https://twitter.com/your_username) - email@example.com

Project Link: [https://github.com/your_username/repo_name](https://github.com/your_username/repo_name)



<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements
* [GitHub Emoji Cheat Sheet](https://www.webpagefx.com/tools/emoji-cheat-sheet)
* [Img Shields](https://shields.io)
* [Choose an Open Source License](https://choosealicense.com)
* [GitHub Pages](https://pages.github.com)
* [Animate.css](https://daneden.github.io/animate.css)
* [Loaders.css](https://connoratherton.com/loaders)
* [Slick Carousel](https://kenwheeler.github.io/slick)
* [Smooth Scroll](https://github.com/cferdinandi/smooth-scroll)
* [Sticky Kit](http://leafo.net/sticky-kit)
* [JVectorMap](http://jvectormap.com)
* [Font Awesome](https://fontawesome.com)





<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/othneildrew/Best-README-Template.svg?style=flat-square
[contributors-url]: https://github.com/othneildrew/Best-README-Template/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/othneildrew/Best-README-Template.svg?style=flat-square
[forks-url]: https://github.com/othneildrew/Best-README-Template/network/members
[stars-shield]: https://img.shields.io/github/stars/othneildrew/Best-README-Template.svg?style=flat-square
[stars-url]: https://github.com/othneildrew/Best-README-Template/stargazers
[issues-shield]: https://img.shields.io/github/issues/othneildrew/Best-README-Template.svg?style=flat-square
[issues-url]: https://github.com/othneildrew/Best-README-Template/issues
[license-shield]: https://img.shields.io/github/license/othneildrew/Best-README-Template.svg?style=flat-square
[license-url]: https://github.com/othneildrew/Best-README-Template/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=flat-square&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/othneildrew
[product-screenshot]: images/screenshot.png
