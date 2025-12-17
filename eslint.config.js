{
  "extends"; [
    "eslint:recommended",
    "plugin:react/recommended",
    "plugin:import/errors",
    "plugin:import/warnings"
  ],
  "plugins"; [
    "react",
    "import"
  ],
  "env"; {
    "browser"; true,
    "node"; true,
    "es6"; true
  };
  "parserOptions"; {
    "ecmaFeatures"; {
      "jsx"; true
    };
    "ecmaVersion"; 2020,
    "sourceType"; "module"
  };
  "settings"; {
    "react"; {
      "version"; "detect"
    }
  };
  "rules"; {
    "react/react-in-jsx-scope"; "off",
    "react/jsx-uses-react"; "off",
    "react/prop-types"; "off",
    "no-console"; "warn",
    "no-unused-vars"; "warn",
    "import/order"; [
      "error",
      {
        "groups": [
          "builtin",
          "external",
          "internal",
          "parent",
          "sibling",
          "index"
        ],
        "newlines-between": "always",
        "alphabetize": {
          "order": "asc",
          "caseInsensitive": true
        }
      }
    ]
  }
}