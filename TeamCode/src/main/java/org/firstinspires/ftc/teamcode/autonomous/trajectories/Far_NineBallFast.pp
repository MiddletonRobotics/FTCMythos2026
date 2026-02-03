{
  "startPoint": {
    "x": 56,
    "y": 6,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-ebdvp3b8nr",
      "name": "Path 1",
      "endPoint": {
        "x": 59.6,
        "y": 18.3,
        "heading": "linear",
        "startDeg": 90,
        "endDeg": 107
      },
      "controlPoints": [],
      "color": "#65A6A5",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "ml627u2r-9i8opx",
      "name": "Path 2",
      "endPoint": {
        "x": 15.2,
        "y": 35.9,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
      },
      "controlPoints": [
        {
          "x": 63.7,
          "y": 41.2
        }
      ],
      "color": "#CBC658",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "ml628vxy-v6w6jt",
      "name": "Path 3",
      "endPoint": {
        "x": 59.6,
        "y": 18.6,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 107
      },
      "controlPoints": [],
      "color": "#B8567B",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "ml62ahib-xtvuze",
      "name": "Path 4",
      "endPoint": {
        "x": 8,
        "y": 9.4,
        "heading": "constant",
        "reverse": false,
        "degrees": 270
      },
      "controlPoints": [
        {
          "x": 27.15514018691589,
          "y": 91.58878504672896
        }
      ],
      "color": "#8D9957",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "ml62do3j-jn1e96",
      "name": "Path 5",
      "endPoint": {
        "x": 59.7,
        "y": 18.1,
        "heading": "linear",
        "reverse": false,
        "startDeg": 270,
        "endDeg": 107
      },
      "controlPoints": [],
      "color": "#DA5D68",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "ml62en32-afi93x",
      "name": "Path 6",
      "endPoint": {
        "x": 37.1,
        "y": 12.2,
        "heading": "linear",
        "reverse": false,
        "startDeg": 107,
        "endDeg": 107
      },
      "controlPoints": [],
      "color": "#679A59",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 144,
          "y": 70
        },
        {
          "x": 144,
          "y": 144
        },
        {
          "x": 120,
          "y": 144
        },
        {
          "x": 138,
          "y": 119
        },
        {
          "x": 138,
          "y": 70
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6,
          "y": 119
        },
        {
          "x": 25,
          "y": 144
        },
        {
          "x": 0,
          "y": 144
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 7,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-ebdvp3b8nr"
    },
    {
      "kind": "path",
      "lineId": "ml627u2r-9i8opx"
    },
    {
      "kind": "path",
      "lineId": "ml628vxy-v6w6jt"
    },
    {
      "kind": "path",
      "lineId": "ml62ahib-xtvuze"
    },
    {
      "kind": "path",
      "lineId": "ml62do3j-jn1e96"
    },
    {
      "kind": "path",
      "lineId": "ml62en32-afi93x"
    }
  ],
  "settings": {
    "xVelocity": 75,
    "yVelocity": 65,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rWidth": 16,
    "rHeight": 16,
    "safetyMargin": 1,
    "maxVelocity": 40,
    "maxAcceleration": 30,
    "maxDeceleration": 30,
    "fieldMap": "decode.webp",
    "robotImage": "/robot.png",
    "theme": "auto",
    "showGhostPaths": false,
    "showOnionLayers": false,
    "onionLayerSpacing": 3,
    "onionColor": "#dc2626",
    "onionNextPointOnly": false
  },
  "version": "1.2.1",
  "timestamp": "2026-02-03T03:57:27.326Z"
}