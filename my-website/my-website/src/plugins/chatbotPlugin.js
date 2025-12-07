const path = require('path');
module.exports = function (context, options) {
  return {
    name: 'docusaurus-chatbot-plugin',
    getClientModules() {
      return [path.resolve(__dirname, './injectChatbot.js')];
    },
  };
};