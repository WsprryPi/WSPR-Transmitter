var Bubblewrap = require("..");
var expect = require("chai").expect;
var assign = require("object-assign");

describe("Bubblewrap", function () {

  var goodData = {
    name: "John Doe",
    age: 30,
    isAlive: true,
    spouse: {name: "Jane Doe"},
    siblings: ["Jack", "Jake"],
    birthdate: new Date()
  };

  var moreGoodData = {
    name: "Joe Doe",
    age: 40,
    isAlive: false,
    spouse: {prop: "val"},
    siblings: ["Alex", "Alexa"],
    birthdate: new Date(0)
  };

  var badData = {
    name: 1234,
    age: "ten",
    isAlive: "true",
    spouse: [],
    siblings: {older: "Jack", younger: "Jake"},
    birthdate: "9/13"
  };

  var schema = {
    age: "number",
    name: "string",
    isAlive: "boolean",
    spouse: "object",
    siblings: "array",
    birthdate: "date"
  };

  function Person (settings) {
    settings = settings || {};
    this.name = settings.name;
    this.age = settings.age;
    this.spouse = settings.spouse;
    this.isAlive = settings.isAlive;
    this.siblings = settings.siblings;
    this.birthdate = settings.birthdate;
  }

  var SafePerson;

  beforeEach(function () {
    SafePerson = Bubblewrap(Person, schema);
  });

  it("works in the simple case", function () {
    var jd = new SafePerson(goodData);
  });

  it("requires that the first argument is a function", function () {
    expect(function () {
      Bubblewrap("asdf", {});
    }).to.throw();
  });

  it("requires that the second argument is an object", function () {
    expect(function () {
      Bubblewrap((function () {}), "asdf");
    }).to.throw();
  });

  it("prevents creating an object with bad data", function () {
    Object.keys(badData).forEach(function (key) {
      var obj = {};
      obj[key] = badData[key];
      var inputData = assign({}, goodData, obj);
      expect(function () {
        new SafePerson(inputData);
      }).to.throw(TypeError);
    });

  });

  it("allows setting good data property on an object", function () {
    Object.keys(moreGoodData).forEach(function (key) {
      var result = new SafePerson(goodData);
      result[key] = moreGoodData[key];
    });
  });

  it("prevents setting an object property with bad data", function () {
    Object.keys(badData).forEach(function (key) {
      var result = new SafePerson(goodData);
      expect(function () {
        result[key] = badData[key];
      }).to.throw(TypeError);
    });
  });

  it("allows defining good data property on an object", function () {
    Object.keys(moreGoodData).forEach(function (key) {
      var result = new SafePerson(goodData);
      Object.defineProperty(result, key, {value: moreGoodData[key]});
    });
  });

  it("prevents defining an object property with bad data", function () {
    Object.keys(badData).forEach(function (key) {
      var result = new SafePerson(goodData);
      expect(function () {
        Object.defineProperty(result, key, {value: badData[key]});
      }).to.throw(TypeError);
    });
  });

  it("prevents deleting required properties of an object", function () {
    Object.keys(badData).forEach(function (key) {
      var result = new SafePerson(goodData);
      expect(function () {
        delete result[key];
      }).to.throw(TypeError);
    });
  });

  it("allows deleting non properties of an object", function () {
    Object.keys(badData).forEach(function (key) {
      var result = new SafePerson(goodData);
      result.extraneous = badData[key];
      delete result.extraneous;
    });
  });

  it("does the same for applying a function (rather than constructing)", function () {
    function Factory (a, b, c) {
      var instance = {};
      instance.a = a;
      instance.b = b;
      instance.c = c;
      return instance;
    }

    var SafeFactory = Bubblewrap(Factory, {
      a: "string",
      b: "number",
      c: "boolean"
    });

    SafeFactory("", 0, false);

    expect(function () {
      SafeFactory(0, "", false);
    }).to.throw();

    expect(function () {
      SafeFactory(0, false, "");
    }).to.throw();

    expect(function () {
      SafeFactory(false, 0, "");
    }).to.throw();

  });

});



describe("#wrap()", function () {
  var wrapped = Bubblewrap.wrap({
    num: 1,
    str: "",
    bool: true,
    arr: [],
    obj: {},
  });

  wrapped.misc = "something";

  expect(function () {
    wrapped.num = false;
  }).to.throw();

  expect(function () {
    wrapped.str = 123456;
  }).to.throw();

  expect(function () {
    wrapped.bool = "asdf";
  }).to.throw();

  expect(function () {
    wrapped.arr = {};
  }).to.throw();

  expect(function () {
    wrapped.obj = [];
  }).to.throw();

});


describe("bad schema", function () {
  it("throws", function () {

    function Stairs (steps) {
      this.steps = steps;
    }

    expect(function () {
      var SafeStars = Bubblewrap(Stairs, {
        steps: "asdf"
      });
    }).to.throw(/isn't a valid type name/);

  });
});