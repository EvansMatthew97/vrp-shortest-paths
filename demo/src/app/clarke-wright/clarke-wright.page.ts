import { Component, ViewChild, ElementRef, OnInit } from '@angular/core';
import { ClarkeWrightProblem, CWWeights } from '../../../../dist';
import { convertLength } from '@turf/helpers';

import * as leaflet from 'leaflet';
import 'leaflet-draw';
import 'leaflet-draw-drag';
import 'leaflet-textpath';

import config from 'src/config';
import { createRandomPoints } from '../../libraries/util';

@Component({
  selector: 'app-clarke-wright',
  templateUrl: './clarke-wright.page.html',
  styleUrls: ['./clarke-wright.page.scss'],
})
export class ClarkeWrightPage implements OnInit {

  @ViewChild('map', { static: true }) mapElement: ElementRef;
  private map: leaflet.Map;

  droneInfo = {
    flightSpeed: 30,
    flightDuration: 15,
  };

  depot = new leaflet.CircleMarker(config.START_POINT as leaflet.LatLngExpression).bindTooltip('Depot');
  drawnItems = new leaflet.FeatureGroup();
  paths = new leaflet.FeatureGroup();

  optimise = true;
  cwWeights: CWWeights = {
    adjacency: 1.0,
    asymmetry: 0.2,
    demand: 0.4,
    minSavings: 0.1,
  };

  randomPointsOptions = {
    count: 100,
    distance: 5,
  };

  constructor() {}

  ngOnInit() {
    // initialise the map
    this.map = leaflet.map(this.mapElement.nativeElement);
    this.map.setView(config.START_POINT as leaflet.LatLngExpression, 13);

    // ensure the map is the right size
    setTimeout(() => this.map.invalidateSize(), 0);

    // add tile layer
    leaflet.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      attribution: '© <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
      opacity: 0.75,
    }).addTo(this.map);

    // drawn items
    this.map.addLayer(this.drawnItems);

    // draw control
    const drawControl = new leaflet.Control.Draw({
      draw: {
        polygon: false,
        rectangle: false,
        circle: false,
        circlemarker: false,
        polyline: false,
      },
      edit: {
        featureGroup: this.drawnItems,
      },
    });
    this.map.addControl(drawControl);

    // add items to layer on draw
    this.map.on(leaflet.Draw.Event.CREATED, (e: any) => {
      this.drawnItems.addLayer(e.layer);
    });

    this.map.on(leaflet.Draw.Event.DELETED, (e: any) => {
      // prevent depot from being deleted      
      for (const i in e.layers._layers) {
        if (e.layers._layers[i] === this.depot) {
          this.drawnItems.addLayer(this.depot);
        }
      }
    });

    // show depot
    this.drawnItems.addLayer(this.depot);

    // show paths
    this.map.addLayer(this.paths);
  }

  calculateSavings() {
    const depot = {
      lon: this.depot.getLatLng().lng,
      lat: this.depot.getLatLng().lat,
    };

    const points = this.drawnItems
      .getLayers()
      .filter(layer => layer !== this.depot)
      .map(layer => {
        const latLng = (layer as any).getLatLng();
        return {
          lon: latLng.lng,
          lat: latLng.lat,
          demand: 1,
        };
      });

    const maxFlightDistance = this.calculateFlightDistance();
    const flightDistanceDegrees = convertLength(maxFlightDistance, 'kilometers', 'degrees');

    const problem = new ClarkeWrightProblem({
      depot: depot,
      customers: points,
      maxDistance: flightDistanceDegrees,
    });

    const solution = problem.solve(this.cwWeights, this.optimise);

    this.paths.clearLayers();

    const colors = [
      'fuchsia',
      'red',
      'orange',
      'green',
      'blue',
      'hotpink',
      'teal',
    ];

    solution.forEach((route, routeIndex) => {
      console.log(route);
      const path = new leaflet.Polyline([
          ...route.points.map(point => new leaflet.LatLng(point[1], point[0])),
          new leaflet.LatLng(route.points[0][1], route.points[0][0]),
        ], {
          color: colors[routeIndex % colors.length],
          weight: 3,
          opacity: 0.5,
        });

      const distance = convertLength(route.distance, 'degrees', 'kilometers');

      (path as any).setText(`${Math.round(distance * 100) / 100}km`, {
        below: true,
        orientation: 'perpendicular',
      });

      this.paths.addLayer(path);
    });
  }

  /**
   * Find flight distance for drone using d = vt
   */
  calculateFlightDistance() {
    return this.droneInfo.flightSpeed * this.droneInfo.flightDuration / 60;
  }

  randomPoints() {
    const points = createRandomPoints(
      this.randomPointsOptions.count,
      [config.START_POINT[1], config.START_POINT[0]],
      this.randomPointsOptions.distance,
    );

    this.drawnItems.getLayers().forEach(layer => {
      if (layer === this.depot) return;
      this.drawnItems.removeLayer(layer);
    });

    points.forEach(point => {
      const marker = new leaflet.CircleMarker([point[1], point[0]], {
        radius: 1,
      });
      this.drawnItems.addLayer(marker);
    });
  }

}
