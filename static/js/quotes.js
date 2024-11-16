
function getRandomQuote(){

    var quotes = [
      "The important thing is not to stop questioning. - Albert Einstein",
      "Innovation distinguishes between a leader and a follower. - Steve Jobs",
      "Science is a way of thinking much more than it is a body of knowledge. - Carl Sagan",
      "The greatest enemy of knowledge is not ignorance, it is the illusion of knowledge. - Stephen Hawking",
      "The best way to predict the future is to invent it. - Alan Kay",
      "Genius is one percent inspiration and ninety-nine percent perspiration. - Thomas Edison",
      "The true sign of intelligence is not knowledge but imagination. - Albert Einstein",
      "Science knows no country, because knowledge belongs to humanity. - Louis Pasteur",
      "Imagination is more important than knowledge. - Albert Einstein",
      "Every great advance in science has issued from a new audacity of imagination. - John Dewey",
      "The good thing about science is that it's true whether or not you believe in it. - Neil deGrasse Tyson",
      "The greatest scientists are artists as well. - Albert Einstein",
      "Science is the great antidote to the poison of enthusiasm and superstition. - Adam Smith",
      "Science is simply common sense at its best. - Thomas Huxley",
      "Science is organized knowledge. Wisdom is organized life. - Immanuel Kant",
      "The most exciting phrase to hear in science, the one that heralds new discoveries, is not 'Eureka!' but 'That's funny...' - Isaac Asimov",
      "Science is the process that takes us from confusion to understanding. - Brian Greene",
      "The first principle is that you must not fool yourself, and you are the easiest person to fool. - Richard Feynman",
      "The true delight is in the finding out rather than in the knowing. - Isaac Asimov",
      "The pursuit of science leads only to the insoluble. - Albert Einstein",
      "The universe is under no obligation to make sense to you. - Neil deGrasse Tyson",
      "Science and everyday life cannot and should not be separated. - Rosalind Franklin",
      "The important thing in science is not so much to obtain new facts as to discover new ways of thinking about them. - William Lawrence Bragg",
      "Make everything as simple as possible, but not simpler. - Albert Einstein"
    ];

    // Get the quote element
    var quoteElement = document.getElementById("quote");

    // Generate a random index to select a quote from the array
    var randomIndex = Math.floor(Math.random() * quotes.length);

    // Set the quote as the content of the quote element
    return quotes[randomIndex];

}
