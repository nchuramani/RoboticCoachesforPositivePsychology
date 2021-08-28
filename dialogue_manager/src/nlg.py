#!/usr/bin/env python

# a dictionary that keeps and the whole script under categories

dialogue = {"general":  [" I couldn't understand what you said. Can you please repeat it?",
                        " Please answer with the requested response."],

            "yes/no":   [["yes", "yeah", "yep", "yeap", "okay", "fine", "true", "ok", "aye", "definitely", "certainly", "exactly", "affirmative", "gladly", "good", "nice", "of course", "positive", "precisely", "sure"],
                         ["no", "nope", "not", "nay", "never", "nae", "naw", "nah"]],

            "emotion": [#sad past and present [0]
                        [" That sounds like a tough experience. I'm sorry.",
                        " I'm sorry that happened to you. That sounds tough.",
                        " I'm sorry that happened. That sounds difficult."],
                        #sad future [1]
                        [" That sounds like it would be a tough experience. I'm sorry.",
                        " I'm sorry. That sounds tough.",
                        " That sounds difficult. I'm sorry about that."],
                        #happy past present [2]
                        [" That sounds great, I'm happy for you.",
                        " That's great, it sounds like a positive experience.",
                        " That's great to hear. I'm happy for you.",
                        " I'm happy to hear that, that sounds good.",
                        " That sounds like a positive thing. I'm happy to hear that."],
                        #happy future [3]
                        [" That sounds great, I'm happy for you.",
                        " That's great, it sounds like it would be a positive experience.",
                        " That's great to hear. I'm happy for you.",
                        " I'm happy to hear that, that sounds good.",
                        " That sounds like a positive thing. I'm happy to hear that."],
                        #angry past present [4]
                        [" That sounds like a tough experience for you. I'm sorry you went through that.",
                        " That sounds really frustrating. I'm sorry to hear that.",
                        " That sounds like a difficult experience for you. I'm sorry you went through that."],
                        #angry future [5]
                        [" That sounds tough. I'm sorry.",
                        " That sounds really frustrating. I'm sorry to hear that.",
                        " That sounds difficult. I'm sorry to hear that."],
                        #fear
                        #[" Wow, that sounds scary.",
                        #" That sounds scary."],
                        #disgust past present [6]
                        [" That sounds like a difficult experience for you.",
                        " That sounds really frustrating.",
                        " I'm sorry you went through that. That sounds difficult."],
                        #disgust future [7]
                        [" That sounds like it would be a difficult experience for you.",
                        " That sounds really frustrating.",
                        " That sounds difficult. I'm sorry."],
                        #Neutral [8]
                        [" Thank you for sharing that with me."]],
                        #surprise
                        #[" Wow, that sounds surprising.",
                        #" Wow, that must have come out of the blue."]],

            "phrases": [[
                # accomplishments past present [0]
                " I'm so pleased. Well done.",
                " You deserve this. I'm happy for you.",
                " Well done. That sounds great.",
                " That sounds great. I'm happy for you",
                " It sounds like you worked hard. Well done."],

                # accomplishments future [1]
                [" I hope you achieve this. That sounds good.",
                 " That sounds like it would be a good accomplishment.",
                 " That sounds great. I hope you achieve this."],

                # gratitude past present [2]
                [" It's great that you could notice this positive thing.",
                 " Paying attention to the positive can help us feel better.",
                 " I'm glad you were able to recognize the gratitude you felt."],

                # gratitude future [3]
                [" I hope you can notice this positive thing happening in the future.",
                 " Paying attention to the positive can help us feel better."]],


            "feedback": [" Thank you for completing the exercises regarding your {}!",
                        " Now I will ask you to provide feedback about how you found this session with me.",
                        " How did you find the exercises? Was it 'good', just 'okay', or was it 'bad'.",
                        " Can you tell me what made the exercises feel that way?",
                        " How well do you think I did as a coach? Was I 'good', just 'okay', or was I 'bad'.",
                        " Could you tell me what made it feel that way for you?",
                        " Thank you for your responses. Now, can you please fill in the survey questionnaires on the tablet in front of you?"],


            "introduction": ["Hi", #Wave
                            " My name is Pepper.",
                            " Nice to meet you!",
                            " We're going to do positive psychology practices today. I'll explain what that means in a minute. Are you okay with that? Please say 'yes' or 'no'.",
                            " Okay. Positive psychology practices aim to guide the participant to focus on the positive things in their life. Often, the negative things in our life get much of our attention, while the positive things are harder to focus on.",
                            " Today, we will first focus on your past, then talk about your present, and then look at the future. With this, we aim to help you think optimistically about your future.",
                            " We will do three exercises each focussing on these time periods. With this, we aim to help you reflect on all three time periods in your life, and cast our attention towards the future.",
                            " We have about 40 minutes together.",
                            " If you have any questions before we begin, I can call in {} to answer them. Do you have any questions? Please say 'yes' or 'no'.",
                            " Great, let's begin then.",
                            " Please come in, {}."],

            "past":     {"impactful": [" First, let's focus on the past. Let's talk about two impactful things that happened in your past, any two things that affected you in the recent past, in the past month or a few weeks. Can you tell me any two impactful things? You can start telling me the first one.",
                                    " If you have trouble coming up with these, remember that it does  not need to be huge, just something impactful. It could have been a rainy or a sunny day in the recent past, or having a brunch with your flatmate, or meeting a friend.",
                                    " How does talking about this to me make you feel now?",
                                    " How do you think this event is affecting you now?",
                                    " Thank you for sharing this with me.",
                                    " Could you tell me about your {} impactful experience you have had in the recent past?"],

                        "grateful": [" That was the end of the first exercise.",
                                    " This next exercise will focus on developing gratitude. Cultivating gratitude can help increase positive affect, subjective happiness and life satisfaction.",
                                    " Let's keep thinking about the past. Please recall two things that you felt grateful for in the past month or a couple of weeks. Could you start with telling me one thing you felt grateful for, big or small?",
                                    " These can be small but positive things that happened, for example having useful feedback about your work, or receiving a gift from a friend. Please, go ahead and tell me.",
                                    [" How did that thing make you feel?",
                                    " How did you notice you were feeling grateful?",
                                    " What significance did this have to you?"],
                                    [" How did the event impact your life?",
                                    " Did you tell someone else about this event and how it made you feel grateful?",
                                    " What difference does noticing this make to you?"],
                                    " Thank you for sharing this with me.",
                                    " Could you tell me another experience that made you feel grateful?"],

                        "accomplishments": [" Now, for our third and final exercise thinking about the past, let's talk about some of your recent accomplishments. Could you tell me two things you have accomplished in the past month or few weeks, big or small? Let's start with the first one.",
                                            " These can be small or big accomplishments in your personal life, at work, or with your friends. For example, something you had been working toward for a while, or something you did for other people, or something that made a positive contribution to you or your life. Please, go ahead and tell me.",
                                            [" How do you think you were able to accomplish that?",
                                            " What personal strengths and qualities do you think you used to accomplish that?",
                                            " Which of your personal skills and qualities helped you accomplish that?"],
                                            [" How did your accomplishment make you feel then?",
                                            " How does your accomplishment affect your feelings now?",
                                            " What difference did achieving this make to you?"],
                                            " Thank you for sharing this with me.",
                                            " Could you tell me about another accomplishment you have had in the recent past?"]},

            "present": {"impactful": [" Now we will move on to talking about the present. Please think about two things that are currently impacting you. They don't need to be positive  or negative, these can be any two things that are affecting you currently. Could you please tell me two impactful things? Please start with the first one.",
                                    " If you have trouble coming up with these, remember that it does  not need to be huge, just something impactful.",
                                    " How does talking about this to make you feel now?",
                                    " How do you think the event is affecting you now?",
                                    " Thank you for sharing this with me.",
                                    " Could you tell me about your {} impactful experience you have had in the recent past?"],
                                    
                        "grateful": [" That was the end of the first exercise.",
                                    " Now let's move to positive psychology practices regarding the present. Let's talk about what you are grateful for right now. What things in your life do you appreciate? Please tell me two things. Let's start with the first one.",
                                    " These can be small but positive things.",
                                    [" How did that thing make you feel?",
                                    " What significance does this have to you?",
                                    " How do you notice gratitude?"],
                                    [" How is this impacting your life?",
                                    " Did you tell someone else about this event and how it made you feel grateful?",
                                    " What difference does noticing this make to you?"],
                                    " Thank you for sharing this with me.",
                                    " Could you tell me another experience that made you feel grateful?"],
                                    
                        "accomplishments": [" Now, let's talk about your present accomplishments. What have you already accomplished today? These can be anything, big or small things. For example eating a pleasant breakfast can be an accomplishment.",
                                            " These can be small accomplishments.",
                                            [" How do you think you were able to accomplish that?",
                                            " What personal strengths and qualities do you think you used to accomplish that?",
                                            " Which of your personal skills and qualities helped you accomplish that?"],
                                            [" How is this accomplishment making you feel?",
                                            " How does this accomplishment affect your feelings now?",
                                            " What difference did achieving this make to you?"],
                                            " Thank you for sharing this with me.",
                                            " Could you tell me about another accomplishment you have had recently?"]},

            "future":   {"impactful": [" That was our last exercise regarding the present. Now, let's move to think about the future. We will do three more exercises thinking about the future before we end our session.",
                                        " What two things do you imagine happening in the future might impact you? It can be something as simple as waking up early in the coming week. Please tell me any two things. Let's start with the first one.",
                                        " If you have trouble coming up with these, remember that it does  not need to be huge, just something impactful. Please, go ahead and tell me.",
                                        " How does talking about this thing potentially happening make you feel now?",
                                        " How would that thing affect you in the future?",
                                        " Thank you for sharing this with me",
                                        " Could you tell me about your {} impactful experience you imagine having?"],

                        "grateful": [" That was the end of the first exercise regarding the future.",
                                    " Now let's move on to the positive psychology practices regarding the future. In positive psychology, we place an emphasis on imagining optimistic futures, which has been shown to reduce pessimism, negative affect, and emotional exhaustion.",
                                    " What two things do you imagine that might happen in the near future, in the next few weeks or the next month, that you would be grateful for? Let's start with the first one.",
                                    " These can be small but positive things that might happen.",
                                    [" How will that make you feel?",
                                    " How will you notice you are feeling grateful?",
                                    " What significance would that have to you?"],
                                    [" How will that impact you?",
                                    " What difference would noticing that thing make to you?"],
                                    " Thank you for sharing this with me.",
                                    " Could you tell me another experience that would make you feel grateful?"],

                        "accomplishments": [" This will be our last exercise.",
                                            " Now, let's think about what accomplishments you would like to make in the future. These can be anything, big or small. You might want to consider something which you've been meaning to do for a while or something that might make a positive contribution to you or your life. Let's start with one.",
                                            " These can be small accomplishments, for example, cooking a tasty meal or reading a good book. ",
                                            [" How do you think you will be able to accomplish that?",
                                            " What personal strengths and qualities do you think you will use to accomplish that?",
                                            " Which of your personal skills and qualities will help you accomplish that?",
                                            " How have you managed to achieve things before? How might you apply these to this new accomplishment?"],
                                            [" How will this accomplishment make you feel?",
                                            " How will this accomplishment affect your feelings in the future?",
                                            " If you achieve this, what difference will it make to you?"],
                                            " Thank you for sharing this with me.",
                                            " Could you tell me about another accomplishment you would like to make in the future?"]},
                                            
            "goodbye":  " Thank you for completing the sessions with me. I hope this was useful for you. {} will now enter the room to ask you a few questions, before you leave. Good Bye!"}